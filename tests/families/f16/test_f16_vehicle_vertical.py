"""Focused vertical proof for the F-16 plug-in's reduced public contract."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx_f16.resources import model_resource_root

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, VehicleCatalogFragment, discover_plugins, plugin_catalog_scope
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    configuration_instance_from_vehicle_request,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionCloseSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)
from taoryx.vehicle_catalog_resources import vehicle_catalog_resources
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_parity_witnesses import (
    load_vehicle_execution_parity_witness_catalog,
    validate_vehicle_execution_parity_witnesses,
)
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog, validate_vehicle_execution_witnesses
from tools.extract_f16_plugin_assets import check as check_f16_plugin_assets
from tools.extract_f16_plugin_assets import extract as extract_f16_plugin_assets

MODEL_ID = "f16_s119"
MODEL_VERSION = "1.0.0+composition-v1"
PACKAGE_ID = "taoryx.f16"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.f16.mission-composition"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
ROOT = Path(__file__).resolve().parents[3]


def test_f16_discovery_defers_source_and_controller_implementations_until_selected() -> None:
    """F-16 discovery advertises controls without importing its plant or screens."""

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(ROOT / "src"),
            str(ROOT / "packages/taoryx-daveml/src"),
            str(ROOT / "packages/taoryx-f16/src"),
        )
    )
    script = """
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

sys.path[:] = [
    entry
    for entry in sys.path
    if "taoryx-reference-models" not in Path(entry).parts
    and "taoryx-reachability" not in Path(entry).parts
]

from taoryx.plugins import DeferredSemanticPreflightHandler, discover_plugins
from taoryx_f16.plugin import PLUGIN


@dataclass(frozen=True)
class EntryPoint:
    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target


deferred = (
    "numpy",
    "scipy",
    "taoryx.f16_composition_episode",
    "taoryx.f16_local_physical_control_screen",
    "taoryx.f16_local_physical_lqi_screen",
    "taoryx.f16_mission_capability",
    "taoryx.f16_mission_translation",
    "taoryx.f16_physical_schedule_interior_screen",
    "taoryx.f16_physical_schedule_transition_screen",
    "taoryx.f16_reduced_batch_episode_parity",
    "taoryx.f16_reduced_execution",
    "taoryx.f16_reduced_interface",
    "taoryx.f16_trim_evidence",
    "taoryx.source_f16",
    "taoryx.trajectory.f16_reduced_adapter",
    "taoryx.trajectory.f16_reductions",
    "taoryx.trajectory.f16_reference",
)
assert not set(deferred) & set(sys.modules)
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.f16", "taoryx_f16.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.f16",)
assert catalog.plugin_revision("taoryx.f16").version == "0.1.0a0"
assert not set(deferred) & set(sys.modules)
assert catalog.contribution("vehicle_interface_extension", "f16_s119").value.family_id == "f16_s119"
assert catalog.contribution("trim_evidence_binding", "reference_f16_s119").value.family_id == "reference_f16_s119"
assert tuple(item.id for item in catalog.records("mission_capability_adapter")) == (
    "taoryx.f16_racetrack.source_route.v1",
    "taoryx.f16_local_physical_control_screen.capability.v1",
    "taoryx.f16_local_physical_surface_lqi_screen.capability.v1",
    "taoryx.f16_local_physical_surface_lqr_schedule_interior_screen.capability.v1",
    "taoryx.f16_local_physical_surface_lqi_schedule_interior_screen.capability.v1",
    "taoryx.f16_local_physical_surface_lqr_schedule_transition_screen.capability.v1",
)
assert all(isinstance(item.value, DeferredSemanticPreflightHandler) for item in catalog.records("semantic_preflight_handler"))
assert not set(deferred) & set(sys.modules)
adapter = catalog.contribution("mission_capability_adapter", "taoryx.f16_racetrack.source_route.v1").value
assert adapter.supports(
    SimpleNamespace(
        family_id="f16_s119",
        mission="powered_fixed_wing_racetrack_v1",
        fidelity="pseudo_6dof",
    )
)
assert "taoryx.f16_mission_capability" in sys.modules
assert "taoryx.f16_mission_translation" in sys.modules
assert "taoryx.source_f16" not in sys.modules
assert "taoryx.f16_reduced_execution" not in sys.modules
assert "taoryx_reference_models" not in sys.modules
assert "taoryx_reachability" not in sys.modules
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    ####


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the F-16 package for this fast vertical contract."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the F-16 provider through the selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF composition witness."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/f16_racetrack_capability_pseudo6dof_compose.yaml")
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused public route tries to expand into an aggregate scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused F-16 execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    import taoryx.batch_episode_parity_dispatch as batch_episode_parity_dispatch
    import taoryx.composition_episode as composition_episode
    import taoryx.mission_capability as mission_capability
    import taoryx.vehicle_batch_execution as vehicle_batch_execution
    import taoryx.vehicle_execution_preflight as vehicle_execution_preflight
    import taoryx.vehicle_interface as vehicle_interface
    import taoryx.vehicle_runtime_lowering as vehicle_runtime_lowering

    monkeypatch.setattr(batch_episode_parity_dispatch, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(composition_episode, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(mission_capability, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_batch_execution, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_execution_preflight, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_interface, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_runtime_lowering, "discover_plugins", unexpected_discovery)
    ####


def test_f16_packaged_data_is_a_current_exact_extract(tmp_path) -> None:
    """The F-16 wheel cannot retain stale assets after a canonical-data change."""

    assert check_f16_plugin_assets() == ()

    temporary_target = tmp_path / "f16-package-data"
    extract_f16_plugin_assets(temporary_target)
    (temporary_target / "obsolete-f16-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_f16_plugin_assets(temporary_target) == ("unexpected generated F-16 asset: obsolete-f16-asset.txt",)
    ####


def test_f16_selected_catalog_declares_and_resolves_its_own_data_fragment(plugins: PluginCatalog) -> None:
    """A focused F-16 host must not silently read sibling catalog fragments."""

    contribution = plugins.contribution("vehicle_catalog_fragment", "taoryx.f16.vehicle-catalog")
    fragment = contribution.value
    assert isinstance(fragment, VehicleCatalogFragment)
    assert fragment.resource_package == "taoryx_f16"
    assert fragment.resource_root == "data"
    assert fragment.family_ids == (MODEL_ID,)

    resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=plugins)
    assert resources == (model_resource_root() / "verification/vehicle_composition_registry.yaml",)
    assert all("taoryx-reference-models" not in source.parts for source in resources)

    parity_resources = vehicle_catalog_resources("verification/vehicle_execution_parity_witnesses.yaml", plugins=plugins)
    assert parity_resources == (model_resource_root() / "verification/vehicle_execution_parity_witnesses.yaml",)
    assert {item.id for item in load_vehicle_execution_witness_catalog(plugins=plugins).witnesses} == {
        "f16-3dof-batch",
        "f16-pseudo6dof-batch",
        "f16-3dof-episode",
        "f16-pseudo6dof-episode",
        "f16-local-direct-wrench-screen-batch",
        "f16-local-surface-screen-batch",
        "f16-local-surface-lqi-screen-batch",
        "f16-local-surface-lqr-schedule-interior-screen-batch",
        "f16-local-surface-lqi-schedule-interior-screen-batch",
        "f16-local-surface-lqr-schedule-transition-screen-batch",
    }
    assert {item.id for item in load_vehicle_execution_parity_witness_catalog(plugins=plugins).witnesses} == {
        "f16-3dof-kinematic-guidance",
        "f16-pseudo6dof-kinematic-guidance",
    }

    with plugin_catalog_scope(plugins):
        assert vehicle_catalog_resources("verification/vehicle_composition_registry.yaml") == resources
        assert {binding.family_id for binding in load_vehicle_execution_binding_catalog().bindings} == {MODEL_ID}
        assert {item.id for item in load_vehicle_execution_parity_witness_catalog().witnesses} == {
            "f16-3dof-kinematic-guidance",
            "f16-pseudo6dof-kinematic-guidance",
        }

    with pytest.raises(FileNotFoundError, match="do not own resource"):
        vehicle_catalog_resources("verification/family_qualification_missions.yaml", plugins=plugins)
    ####


def test_f16_focused_provider_advertises_versioned_reduced_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Discovery, model metadata, and authoring expose only the intended reduced API."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-f16"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64

    extension = plugins.contribution("vehicle_interface_extension", MODEL_ID)
    assert extension.plugin.id == PACKAGE_ID
    assert extension.value.family_id == MODEL_ID

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)

    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64
    assert model.common_runner_operations == ("batch", "step")

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["default_authority_id"] == "kinematic_guidance"
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "kinematic_guidance",
        "reduced_pilot_command",
        "body_rate_command",
        "live_waypoint_guidance",
    }
    channels = cast(list[dict[str, object]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
    }
    available = cast(list[dict[str, object]], controller["available_channels"])
    assert not any(str(item["id"]).startswith("effector.") for item in available)
    ####


def test_f16_composition_compiler_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Direct composition compilation honors the caller's focused catalog."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/f16_racetrack_capability_pseudo6dof_compose.yaml")
    _forbid_catalog_rediscovery(monkeypatch)

    catalog = load_resolved_vehicle_composition_catalog(plugins=plugins)
    assert tuple(item.family.family_id for item in catalog.vehicles) == (MODEL_ID,)

    composition = compile_vehicle_composition(request, catalog=catalog, plugins=plugins)

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.observation.profile_id == "truth_debug"
    ####


def test_f16_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The common runner must use the F-16 catalog instead of rediscovering every plug-in."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="f16-focused-vertical-batch",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            output=MissionCompositionOutputSelection(mode="core", maximum_samples_per_object=3),
        )
    )

    assert response.kind == "trajectory", response.model_dump(mode="json", by_alias=True)
    assert response.result.status == "completed"
    assert response.result.provider_version == PACKAGE_VERSION
    assert [(item.model_id, item.fidelity, item.realization_id) for item in response.result.objects] == [
        (MODEL_ID, "pseudo_6dof", "pseudo_6dof"),
    ]
    assert len(response.result.objects[0].samples) == 3
    ####


def test_f16_common_session_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The stateful API uses the same focused scope and exposes body-rate readback."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="f16-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            authority_profile_id="body_rate_command",
            seed=13,
            integration_step_s=0.02,
        )
    )

    assert descriptor.active_authority_profile_id == "body_rate_command"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id for item in descriptor.action_schema} == {
        "propulsion.command.fraction",
        "body_rate.roll.command",
        "body_rate.pitch.command",
        "body_rate.yaw.command",
    }
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"body_rate.roll.command": 0.05},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    feedback = {item.channel_id: item for item in stepped.control_feedback}
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert feedback["body_rate.roll.command"].disposition == "applied_as_requested"
    assert feedback["body_rate.roll.command"].achievement_status == "observed"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####


def test_f16_reduced_witness_and_parity_gates_do_not_expand_to_other_plugins(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The quick gate covers only reduced F-16 routes under one exact scope."""

    _forbid_catalog_rediscovery(monkeypatch)
    witnesses = validate_vehicle_execution_witnesses(
        witness_ids=(
            "f16-3dof-batch",
            "f16-pseudo6dof-batch",
            "f16-3dof-episode",
            "f16-pseudo6dof-episode",
        ),
        plugins=plugins,
    )
    assert witnesses["status"] == "pass", witnesses
    assert witnesses["witness_count"] == 4
    assert witnesses["runnable_binding_count"] == 4
    assert witnesses["variant_witness_count"] == 0
    assert witnesses["graph_extension_witness_count"] == 0

    parity = validate_vehicle_execution_parity_witnesses(
        family_ids=(MODEL_ID,),
        plugins=plugins,
    )
    assert parity["status"] == "pass", parity
    assert parity["registered_binding_count"] == 2
    assert parity["witness_count"] == 2
    ####
