"""Focused vertical proof for the standalone dual-launch workflow plug-in."""

from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Literal, cast

import pytest
from taoryx.trajectory.dual_launch_mission_composition import build_dual_launch_example_configuration

import taoryx.mission_workflow_endpoint as workflow_endpoints
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import MissionWorkflowEndpointCatalogFragment, PluginCatalog, discover_plugins
from taoryx.trajectory.execution_contract import (
    MissionCompositionOutputSelection,
    MissionCompositionRunRequest,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)
from tools.extract_dual_launch_plugin_assets import check as check_dual_launch_plugin_assets
from tools.extract_dual_launch_plugin_assets import extract as extract_dual_launch_plugin_assets

PACKAGE_ID = "taoryx.dual-launch"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.dual-launch.mission-composition"
MODEL_ID = "dual_launch_glider"
MODEL_VERSION = "1.0.0-mission-composition-v1"
FIDELITY = "point_mass_3dof"
REALIZATION_ID = "generated_native_problem"
_WORKFLOW_ENDPOINT_ID = "dual-launch-attached-booster-batch"
EXPECTED_CHANNELS = {
    "position.local.north",
    "position.local.east",
    "position.local.down",
    "velocity.local.north",
    "velocity.local.east",
    "velocity.local.down",
    "mass.total",
    "propulsion.thrust",
    "propulsion.mass_flow",
    "control.bank.commanded",
    "control.throttle.commanded",
    "aerodynamics.dynamic_pressure",
    "diagnostics.segment_index",
}


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the dual-launch package for this fast workflow proof."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the selected provider without widening plug-in discovery."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared(provider: Any, launch_mode: Literal["air_release", "attached_booster"]) -> Any:
    """Build an exact launch-form request through the public package schema."""

    return provider.validate_configuration(
        build_dual_launch_example_configuration(
            launch_mode,
            provider.get_model_schema(MODEL_ID),
        )
    )
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused workflow route falls back to aggregate discovery."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused dual-launch execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    monkeypatch.setattr(workflow_endpoints, "discover_plugins", unexpected_discovery)
    ####


def test_dual_launch_packaged_data_is_a_current_exact_extract(tmp_path: Path) -> None:
    """The Dual Launch wheel cannot retain stale workflow assets."""

    assert check_dual_launch_plugin_assets() == ()

    temporary_target = tmp_path / "dual-launch-package-data"
    extract_dual_launch_plugin_assets(temporary_target)
    (temporary_target / "obsolete-dual-launch-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_dual_launch_plugin_assets(temporary_target) == ("unexpected generated Dual Launch asset: obsolete-dual-launch-asset.txt",)
    ####


def test_dual_launch_focused_provider_advertises_versioned_batch_only_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The selected package owns its model API, controls, and endpoint metadata."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-dual-launch"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    owned_contributions = {
        (contribution.kind, contribution.id)
        for contribution in plugins.contributions
        if contribution.plugin.id == PACKAGE_ID
    }
    assert revision.contribution_count == len(owned_contributions)
    assert {
        ("mission_composition_provider", PROVIDER_ID),
        ("mission_workflow_endpoint_catalog", "taoryx.dual-launch.workflow-endpoints"),
    } <= owned_contributions
    assert len(revision.fingerprint) == 64
    contribution = plugins.contribution(
        "mission_workflow_endpoint_catalog",
        "taoryx.dual-launch.workflow-endpoints",
    )
    fragment = contribution.value
    assert contribution.plugin.id == PACKAGE_ID
    assert isinstance(fragment, MissionWorkflowEndpointCatalogFragment)
    assert fragment.endpoint_ids == (_WORKFLOW_ENDPOINT_ID,)

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    native_provider = provider.resolve_provider()
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)
    assert native_provider.plugin_catalog is plugins
    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert model.common_runner_operations == ("batch", "step")
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=REALIZATION_ID,
        mission_template_id="attached_booster_waypoint",
    )
    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    assert selection["physical_family"] == "glider"
    data_contract = cast(dict[str, object], plan["data_contract"])
    assert set(cast(list[str], data_contract["core_output_channels"])) == {
        channel for channel in EXPECTED_CHANNELS if channel.startswith(("position.", "velocity."))
    }
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "provider_managed"
    assert controller["control_status"] == "internally_generated"
    assert {item["id"] for item in cast(list[dict[str, Any]], controller["channels"])} == {
        "command.bank",
        "command.throttle",
    }
    assert controller["campaigns"] == []
    endpoint = cast(dict[str, object], plan["focused_endpoint_verification"])
    assert endpoint["selected_endpoint_ids"] == [_WORKFLOW_ENDPOINT_ID]
    ####


@pytest.mark.parametrize(
    ("launch_mode", "mission_id", "maximum_samples"),
    (
        ("air_release", "air_release_waypoint", 1),
        ("attached_booster", "attached_booster_waypoint", 17),
    ),
)
def test_each_launch_form_executes_through_the_selected_package_runner(
    plugins: PluginCatalog,
    launch_mode: Literal["air_release", "attached_booster"],
    mission_id: str,
    maximum_samples: int,
) -> None:
    """Both forms return finite normalized telemetry and lifecycle evidence."""

    provider = _provider(plugins)
    prepared = _prepared(provider, launch_mode)
    response = provider.build_runner().run(
        MissionCompositionRunRequest(
            request_id=f"dual-launch-focused-vertical-{launch_mode}",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=maximum_samples),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    result = response.result
    assert result.status == "completed"
    assert result.provider_version == PACKAGE_VERSION
    assert result.primary_model_id == MODEL_ID
    assert len(result.objects) == 1
    assert len(result.relationships) == 0
    primary = result.objects[0]
    assert primary.realization_id == REALIZATION_ID
    assert primary.fidelity == FIDELITY
    assert len(primary.samples) == maximum_samples
    assert {item.id for item in primary.channels} == EXPECTED_CHANNELS
    assert all(set(item.values) == EXPECTED_CHANNELS for item in primary.samples)
    assert all(math.isfinite(float(item.values["mass.total"])) for item in primary.samples)
    assert all(math.isfinite(float(item.values["velocity.local.down"])) for item in primary.samples)
    assert all(0.0 <= float(item.values["control.throttle.commanded"]) <= 1.0 for item in primary.samples)

    model = provider.model(MODEL_ID)
    mission = next(item for item in model.mission_templates if item.id == mission_id)
    batch = next(item for item in mission.operations if item.fidelity == FIDELITY and item.operation == "batch")
    assert batch.status == "available"
    assert batch.common_runner_status == "registered"
    if launch_mode == "attached_booster":
        separation = next(item for item in result.events if item.category == "separation")
        assert separation.kind == "booster-glider-separation"
        assert separation.parent_object_id is None
    else:
        assert not [item for item in result.events if item.category == "separation"]
    ####


def test_dual_launch_uses_a_read_only_core_replay_session_without_child_fabrication(plugins: PluginCatalog) -> None:
    """The common step lifecycle replays truth without inventing controls or children."""

    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    attached = next(item for item in model.mission_templates if item.id == "attached_booster_waypoint")
    point_step = next(item for item in attached.operations if item.fidelity == FIDELITY and item.operation == "step")
    assert point_step.status == "available"
    assert point_step.common_runner_status == "registered"
    assert point_step.execution_mode == "core_batch_replay"
    assert point_step.blockers == ()
    assert attached.compatible_fidelities == (FIDELITY,)
    assert not [item for item in attached.operations if item.fidelity == "pseudo_6dof"]
    pseudo = next(item for item in model.fidelities if item.id == "pseudo_6dof")
    assert pseudo.operations == ("validate",)
    assert pseudo.blockers == ("no native pseudo_or_rigid_body_dual_launch_executor",)
    deployment = model.deployments[0]
    assert deployment.status == "declared"
    assert deployment.lifecycle == "event_only"
    assert deployment.blockers == ("independent_parent_and_child_trajectory_binding",)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="dual-launch-focused-workflow-boundary",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=_prepared(provider, "attached_booster"),
            integration_step_s=0.2,
        )
    )
    assert descriptor.state_owner == "core_batch_replay_session"
    assert descriptor.timestep_semantics == "caller_duration_advanced_to_next_replay_sample"
    assert descriptor.action_schema == ()
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            duration_s=0.2,
        )
    )
    assert stepped.time_end_s > stepped.time_start_s
    assert len(stepped.observation.standard_ecef.ecef_from_body_wxyz) == 4
    ####


def test_dual_launch_endpoint_stays_selected_and_has_a_package_data_fallback(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The endpoint witness follows the selected package in source and wheel modes."""

    _forbid_catalog_rediscovery(monkeypatch)
    source_catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert [item.id for item in source_catalog.endpoints] == [_WORKFLOW_ENDPOINT_ID]
    missing_canonical = Path("/tmp/taoryx-dual-launch-no-canonical-workflow-catalog.yaml")
    monkeypatch.setattr(workflow_endpoints, "MISSION_WORKFLOW_ENDPOINT_SPECS", missing_canonical)
    catalog = workflow_endpoints.load_mission_workflow_endpoint_catalog(plugins=plugins)
    assert [item.id for item in catalog.endpoints] == [_WORKFLOW_ENDPOINT_ID]
    report = workflow_endpoints.verify_mission_workflow_endpoint(
        _WORKFLOW_ENDPOINT_ID,
        execute=True,
        plugins=plugins,
    )
    assert report["status"] == "pass", report
    assert report["records"]["advertisement"]["provider_version"] == PACKAGE_VERSION  # type: ignore[index]
    ####


def test_dual_launch_source_tree_needs_neither_aggregate_nor_simple_aero() -> None:
    """A focused source host resolves only this package's provider and data."""

    root = Path(__file__).resolve().parents[3]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-trajectory-contracts/src",
            "packages/taoryx-dual-launch/src",
            "packages/taoryx-reference-models/src",
            "packages/taoryx-simple-aero/src",
        )
    )
    script = """
import sys
from pathlib import Path

# Keep the former aggregate and neighboring workflow source roots visible so
# selected discovery proves it did not rely on their absence.
sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part.startswith("taoryx-")
        and part
        not in {
            "taoryx-trajectory-contracts",
            "taoryx-dual-launch",
            "taoryx-reference-models",
            "taoryx-simple-aero",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.dual-launch",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.dual-launch",)
assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
    "taoryx.dual-launch.mission-composition",
)
assert tuple(item.id for item in catalog.records("mission_workflow_endpoint_catalog")) == (
    "taoryx.dual-launch.workflow-endpoints",
)
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx_simple_aero.plugin" not in sys.modules
assert "taoryx.trajectory.registry_mission_composition" not in sys.modules

provider = catalog.build_mission_composition_provider_registry().provider(
    "taoryx.dual-launch.mission-composition"
)
assert provider.metadata.version == "0.1.0a0"
assert tuple(item.id for item in provider.list_models()) == ("dual_launch_glider",)
from taoryx.trajectory.dual_launch_mission_composition import build_dual_launch_example_configuration
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest

prepared = provider.validate_configuration(
    build_dual_launch_example_configuration("attached_booster", provider.get_model_schema("dual_launch_glider"))
)
response = provider.build_runner().run(
    MissionCompositionRunRequest(
        request_id="dual-launch-source-isolation",
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        prepared_configuration=prepared,
        output=MissionCompositionOutputSelection(mode="all", maximum_samples_per_object=1),
    )
)
assert response.result.status == "completed"
assert "taoryx_reference_models" not in sys.modules
assert "taoryx_simple_aero" not in sys.modules
assert "taoryx.trajectory.registry_mission_composition" not in sys.modules
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=root,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    ####
