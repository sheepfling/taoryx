"""Focused vertical proof for the Hummingbird plug-in's reduced public contract."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx_hummingbird.resources import model_resource_root

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
    MissionCompositionSwitchAuthorityRequest,
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
from tools.extract_hummingbird_plugin_assets import check as check_hummingbird_plugin_assets
from tools.extract_hummingbird_plugin_assets import extract as extract_hummingbird_plugin_assets

MODEL_ID = "hummingbird"
MODEL_VERSION = "1.0.0+composition-v1"
PACKAGE_ID = "taoryx.hummingbird"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.hummingbird.mission-composition"
MISSION_ID = "multirotor_pad_box_yaw_recovery_land_v1"
ROOT = Path(__file__).resolve().parents[3]


def test_hummingbird_discovery_defers_direct_wrench_configuration_and_numerics() -> None:
    """Static direct-wrench metadata must not construct a source-hover runtime."""

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(ROOT / "src"),
            str(ROOT / "packages/taoryx-hummingbird/src"),
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
from taoryx_hummingbird.plugin import PLUGIN


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
    "taoryx.aero_drag_tables",
    "taoryx.composition_batch_episode_parity",
    "taoryx.hummingbird_composition_episode",
    "taoryx.hummingbird_composition_execution",
    "taoryx.hummingbird_local_physical_control_screen",
    "taoryx.hummingbird_mission_capability",
    "taoryx.hummingbird_mission_translation",
    "taoryx.hummingbird_preflight",
    "taoryx.hummingbird_reduced_interface",
    "taoryx.local_direct_wrench",
    "taoryx.local_direct_wrench_mission_translation",
    "taoryx.source_table_multirotor",
    "taoryx.trajectory.hummingbird_3dof",
    "taoryx.trajectory.hummingbird_adapter",
    "taoryx.trajectory.hummingbird_pseudo6dof",
)
assert not set(deferred) & set(sys.modules)
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.hummingbird", "taoryx_hummingbird.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.hummingbird",)
assert catalog.plugin_revision("taoryx.hummingbird").version == "0.1.0a0"
assert not set(deferred) & set(sys.modules)
assert tuple(item.id for item in catalog.records("mission_capability_adapter")) == (
    "taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
    "taoryx.multirotor_hover_translation.capability.v1",
    "taoryx.hummingbird.local_individual_rotor_lqi_screen.capability.v1",
    "taoryx.hummingbird.local_horizontal_translation_lqi_screen.capability.v1",
    "taoryx.hummingbird.local_vertical_translation_lqi_screen.capability.v1",
)
assert all(isinstance(item.value, DeferredSemanticPreflightHandler) for item in catalog.records("semantic_preflight_handler"))
screens = catalog.build_local_direct_wrench_screen_registry()
advertisement = screens.advertisement(
    family_id="hummingbird",
    mission_id="hummingbird_local_direct_wrench_screen_v1",
    fidelity="rigid_body_6dof_direct_wrench",
)
assert advertisement is not None
assert advertisement["control_realization"] == "direct_wrench"
assert not set(deferred) & set(sys.modules)
adapter = catalog.contribution(
    "mission_capability_adapter",
    "taoryx.hummingbird.local_direct_wrench_screen.capability.v1",
).value
assert adapter.supports(
    SimpleNamespace(
        family_id="hummingbird",
        mission="hummingbird_local_direct_wrench_screen_v1",
        initialization=SimpleNamespace(id="source_individual_rotor_hover_local_point"),
        fidelity="rigid_body_6dof_direct_wrench",
    )
)
assert not set(deferred) & set(sys.modules)
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
    """Discover only the Hummingbird package for this fast vertical contract."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the Hummingbird provider through the selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF composition witness."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused public route tries to expand into an aggregate scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused Hummingbird execution unexpectedly rediscovered the aggregate plug-in catalog")
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


def test_hummingbird_packaged_data_is_a_current_exact_extract(tmp_path) -> None:
    """The Hummingbird wheel cannot retain stale assets after a source-data change."""

    assert check_hummingbird_plugin_assets() == ()

    temporary_target = tmp_path / "hummingbird-package-data"
    extract_hummingbird_plugin_assets(temporary_target)
    (temporary_target / "obsolete-hummingbird-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_hummingbird_plugin_assets(temporary_target) == ("unexpected generated Hummingbird asset: obsolete-hummingbird-asset.txt",)
    ####


def test_hummingbird_selected_catalog_declares_and_resolves_its_own_data_fragment(plugins: PluginCatalog) -> None:
    """A focused Hummingbird host must not silently read sibling fragments."""

    contribution = plugins.contribution("vehicle_catalog_fragment", "taoryx.hummingbird.vehicle-catalog")
    fragment = contribution.value
    assert isinstance(fragment, VehicleCatalogFragment)
    assert fragment.resource_package == "taoryx_hummingbird"
    assert fragment.resource_root == "data"
    assert fragment.family_ids == (MODEL_ID,)

    resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=plugins)
    assert resources == (model_resource_root() / "verification/vehicle_composition_registry.yaml",)
    assert all("taoryx-reference-models" not in source.parts for source in resources)
    assert tuple(item.family.family_id for item in load_resolved_vehicle_composition_catalog(plugins=plugins).vehicles) == (MODEL_ID,)

    parity_resources = vehicle_catalog_resources("verification/vehicle_execution_parity_witnesses.yaml", plugins=plugins)
    assert parity_resources == (model_resource_root() / "verification/vehicle_execution_parity_witnesses.yaml",)
    witnesses = load_vehicle_execution_witness_catalog(plugins=plugins)
    assert {item.id for item in witnesses.witnesses} == {
        "hummingbird-pseudo6dof-batch",
        "hummingbird-pseudo6dof-episode",
        "hummingbird-local-physical-lqi-screen-batch",
        "hummingbird-local-horizontal-translation-lqi-screen-batch",
        "hummingbird-local-vertical-translation-lqi-screen-batch",
        "hummingbird-local-direct-wrench-screen-batch",
    }
    assert {item.id for item in witnesses.variant_witnesses} == {"hummingbird-grounded-mass-pseudo6dof"}
    assert {item.id for item in witnesses.graph_extension_witnesses} == {"hummingbird-timeout-recovery-graph-pseudo6dof"}
    assert {item.id for item in load_vehicle_execution_parity_witness_catalog(plugins=plugins).witnesses} == {
        "hummingbird-pseudo6dof-body-motion",
    }

    with plugin_catalog_scope(plugins):
        assert vehicle_catalog_resources("verification/vehicle_composition_registry.yaml") == resources
        assert {binding.family_id for binding in load_vehicle_execution_binding_catalog().bindings} == {MODEL_ID}
        assert {item.id for item in load_vehicle_execution_witness_catalog().graph_extension_witnesses} == {
            "hummingbird-timeout-recovery-graph-pseudo6dof",
        }
        assert {item.id for item in load_vehicle_execution_parity_witness_catalog().witnesses} == {
            "hummingbird-pseudo6dof-body-motion",
        }

    with pytest.raises(FileNotFoundError, match="do not own resource"):
        vehicle_catalog_resources("verification/family_qualification_missions.yaml", plugins=plugins)
    ####


def test_hummingbird_focused_provider_advertises_versioned_reduced_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Discovery, model metadata, and authoring expose only the reduced API."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-hummingbird"
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
    assert controller["default_authority_id"] == "body_motion_response"
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {
        "body_motion_response",
        "velocity_yaw_command",
        "live_waypoint_guidance",
    }
    channels = cast(list[dict[str, object]], controller["channels"])
    assert {item["id"] for item in channels} == {
        "attitude.roll.command",
        "attitude.pitch.command",
        "attitude.yaw.command",
        "propulsion.command.fraction",
        "propulsion.enable",
    }
    available = cast(list[dict[str, object]], controller["available_channels"])
    assert not any(str(item["id"]).startswith("effector.") for item in available)
    ####


def test_hummingbird_composition_compiler_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Direct composition compilation honors the caller's focused catalog."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    _forbid_catalog_rediscovery(monkeypatch)

    composition = compile_vehicle_composition(request, plugins=plugins)

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == "pseudo_6dof"
    assert composition.observation.profile_id == "truth_debug"
    ####


def test_hummingbird_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The common runner uses the Hummingbird catalog without sibling discovery."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="hummingbird-focused-vertical-batch",
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


def test_hummingbird_common_session_route_retains_scope_and_can_switch_authority(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The session API exposes introspectable velocity and waypoint controls."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="hummingbird-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            authority_profile_id="velocity_yaw_command",
            seed=13,
            integration_step_s=0.02,
        )
    )

    assert descriptor.active_authority_profile_id == "velocity_yaw_command"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id for item in descriptor.action_schema} == {
        "velocity.north.command",
        "velocity.east.command",
        "velocity.vertical.command",
        "attitude.yaw.command",
        "propulsion.enable",
    }
    assert descriptor.initial_observation.values["guidance.waypoint.status"] == "inactive"

    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"velocity.north.command": 0.5},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    feedback = {item.channel_id: item for item in stepped.control_feedback}
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert feedback["velocity.north.command"].disposition == "applied_as_requested"
    assert feedback["velocity.north.command"].achievement_status == "observed"

    transition = manager.switch_authority(
        MissionCompositionSwitchAuthorityRequest(
            session_id=descriptor.session_id,
            authority_profile_id="live_waypoint_guidance",
            expected_sequence=1,
        )
    )
    assert transition.active_authority_profile_id == "live_waypoint_guidance"
    assert {item.id for item in transition.action_schema} == {
        "navigation.waypoint.north.command",
        "navigation.waypoint.east.command",
        "navigation.waypoint.altitude.command",
        "navigation.waypoint.capture_radius.command",
        "navigation.waypoint.speed.command",
        "navigation.waypoint.vertical_speed.command",
        "attitude.yaw.command",
        "propulsion.enable",
    }
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####


def test_hummingbird_reduced_witness_and_parity_gates_do_not_expand_to_other_plugins(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The quick gate covers the pseudo path without physical rotor replays."""

    _forbid_catalog_rediscovery(monkeypatch)
    witnesses = validate_vehicle_execution_witnesses(
        witness_ids=("hummingbird-pseudo6dof-batch", "hummingbird-pseudo6dof-episode"),
        plugins=plugins,
    )
    assert witnesses["status"] == "pass", witnesses
    assert witnesses["witness_count"] == 2
    assert witnesses["runnable_binding_count"] == 2
    assert witnesses["variant_witness_count"] == 0
    assert witnesses["graph_extension_witness_count"] == 0

    parity = validate_vehicle_execution_parity_witnesses(
        family_ids=(MODEL_ID,),
        plugins=plugins,
    )
    assert parity["status"] == "pass", parity
    assert parity["registered_binding_count"] == 1
    assert parity["witness_count"] == 1
    ####
