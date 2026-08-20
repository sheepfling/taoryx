"""Focused vertical proof for the passive tumbling-body plug-in."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx_passive_bodies.resources import model_resource_root

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import PluginCatalog, discover_plugins
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
from taoryx.trajectory.native_mission_composition import (
    build_registry_mission_composition_runner,
    configuration_instance_from_vehicle_request,
)
from taoryx.trajectory.session_contract import (
    MissionCompositionOpenSessionRequest,
    MissionCompositionSessionManager,
    MissionCompositionSessionStepRequest,
)
from taoryx.vehicle_composition import (
    compile_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_execution_bindings import load_vehicle_batch_episode_parity_catalog
from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses
from tools.extract_passive_bodies_plugin_assets import check as check_passive_bodies_plugin_assets
from tools.extract_passive_bodies_plugin_assets import extract as extract_passive_bodies_plugin_assets

MODEL_ID = "tumbling_body"
MODEL_VERSION = "1.0.0+composition-v1"
PACKAGE_ID = "taoryx.passive-bodies"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.passive-bodies.mission-composition"
MISSION_ID = "tumbling_body_release_damping_impact_v1"
_CHILD_RUNTIME_ID = "taoryx.passive-bodies.local-atmosphere-release.v1"
ROOT = Path(__file__).resolve().parents[3]


def test_passive_discovery_defers_release_implementations_until_their_contract_is_used() -> None:
    """Discovery retains only stable passive identities, never the child propagator."""

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(ROOT / "src"),
            str(ROOT / "packages/taoryx-passive-bodies/src"),
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
from taoryx_passive_bodies.plugin import PLUGIN


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
    "taoryx.family_adapter",
    "taoryx.family_adapter_registry",
    "taoryx_passive_bodies.capabilities",
    "taoryx_passive_bodies.preflight",
    "taoryx_passive_bodies.deployment_runtime",
    "taoryx.passive_body_dynamics",
    "taoryx.passive_body_interface",
    "taoryx.passive_tumbling_composition_execution",
    "taoryx.passive_tumbling_mission_translation",
)
assert not set(deferred) & set(sys.modules)
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.passive-bodies", "taoryx_passive_bodies.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.passive-bodies",)
assert catalog.plugin_revision("taoryx.passive-bodies").version == "0.1.0a0"
assert not set(deferred) & set(sys.modules)
assert tuple(item.id for item in catalog.records("mission_capability_adapter")) == (
    "taoryx.passive_tumbling_release.capability.v1",
)
assert all(isinstance(item.value, DeferredSemanticPreflightHandler) for item in catalog.records("semantic_preflight_handler"))
runtime = catalog.build_deployment_child_runtime_registry().resolve(
    "taoryx.passive-bodies.local-atmosphere-release.v1"
)
assert runtime.id == "taoryx.passive-bodies.local-atmosphere-release.v1"
assert not set(deferred) & set(sys.modules)
adapter = catalog.contribution("mission_capability_adapter", "taoryx.passive_tumbling_release.capability.v1").value
assert adapter.supports(
    SimpleNamespace(
        family_id="tumbling_body",
        mission="tumbling_body_release_damping_impact_v1",
        fidelity="pseudo_6dof",
    )
)
assert "taoryx_passive_bodies.capabilities" in sys.modules
assert "taoryx_passive_bodies.preflight" not in sys.modules
assert runtime.supports(
    SimpleNamespace(
        child=SimpleNamespace(
            plugin_id="taoryx.passive-bodies",
            runtime_id="taoryx.passive-bodies.local-atmosphere-release.v1",
            model_id="nesc-synthetic-cylinder",
            family_id="tumbling_body",
            fidelity="pseudo_6dof",
        )
    )
)
assert "taoryx_passive_bodies.deployment_runtime" in sys.modules
assert "taoryx.passive_body_dynamics" in sys.modules
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
    """Discover only the passive package for this fast vertical contract."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the passive provider through the selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF direct-release witness."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml")
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused public route tries to expand into an aggregate scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused passive-body execution unexpectedly rediscovered the aggregate plug-in catalog")
        ####

    import taoryx.composition_episode as composition_episode
    import taoryx.mission_capability as mission_capability
    import taoryx.vehicle_batch_execution as vehicle_batch_execution
    import taoryx.vehicle_execution_preflight as vehicle_execution_preflight
    import taoryx.vehicle_interface as vehicle_interface
    import taoryx.vehicle_runtime_lowering as vehicle_runtime_lowering

    monkeypatch.setattr(composition_episode, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(mission_capability, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_batch_execution, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_execution_preflight, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_interface, "discover_plugins", unexpected_discovery)
    monkeypatch.setattr(vehicle_runtime_lowering, "discover_plugins", unexpected_discovery)
    ####


def test_passive_bodies_packaged_data_is_a_current_exact_extract(tmp_path) -> None:
    """The passive-bodies wheel cannot retain stale direct-release assets."""

    assert check_passive_bodies_plugin_assets() == ()

    temporary_target = tmp_path / "passive-bodies-package-data"
    extract_passive_bodies_plugin_assets(temporary_target)
    (temporary_target / "obsolete-passive-body-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_passive_bodies_plugin_assets(temporary_target) == ("unexpected generated passive-body asset: obsolete-passive-body-asset.txt",)
    ####


def test_passive_provider_advertises_versioned_read_only_replay_contract(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The focused provider replays batch truth through the standard no-control session."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-passive-bodies"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64
    assert plugins.contribution("vehicle_interface_extension", MODEL_ID).plugin.id == PACKAGE_ID
    assert plugins.build_deployment_child_runtime_registry().ids == (_CHILD_RUNTIME_ID,)

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)

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
        fidelity="pseudo_6dof",
        realization_id="pseudo_6dof",
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["status"] == "not_applicable"
    assert controller["channels"] == []
    assert controller["campaigns"] == []
    assert controller["default_authority_id"] is None
    assert cast(list[dict[str, object]], controller["authorities"])[0]["id"] == "no_external_action"

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="passive-release-replay-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=_prepared_configuration(provider),
            integration_step_s=0.2,
        )
    )
    assert descriptor.action_schema == ()
    assert descriptor.state_owner == "core_batch_replay_session"
    assert descriptor.timestep_semantics == "caller_duration_advanced_to_next_replay_sample"
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            duration_s=0.2,
        )
    )
    assert stepped.time_end_s > stepped.time_start_s
    assert len(stepped.observation.standard_ecef.ecef_from_body_wxyz) == 4

    composition = compile_vehicle_composition(
        load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml"),
        plugins=plugins,
    )
    interface = resolve_vehicle_composition_interface_contract(composition, plugins=plugins)
    assert interface.action_channels == ()
    assert {item.id for item in interface.authority_profiles} == {"no_external_action"}
    status = {item.id: item for item in interface.status_channels}
    assert {"aerodynamics.drag_force", "aerodynamics.projected_area", "angular_rate.norm"} <= set(status)
    assert all(status[item].availability == "available_in_batch" for item in status)
    assert interface.resource_channels[0].id == "resources.mass.total"
    ####


def test_passive_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The normal batch API uses only the selected passive package catalog."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="passive-focused-vertical-batch",
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


def test_passive_reduced_witnesses_are_scoped_and_explicitly_have_no_parity_claim(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The direct-release gate stays batch-only instead of inventing equivalence."""

    _forbid_catalog_rediscovery(monkeypatch)
    witnesses = validate_vehicle_execution_witnesses(
        witness_ids=("tumbling-3dof-batch", "tumbling-pseudo6dof-batch"),
        plugins=plugins,
    )
    assert witnesses["status"] == "pass", witnesses
    assert witnesses["witness_count"] == 2
    assert witnesses["runnable_binding_count"] == 2
    assert witnesses["variant_witness_count"] == 0
    assert witnesses["graph_extension_witness_count"] == 0

    parity = load_vehicle_batch_episode_parity_catalog()
    assert not any(binding.family_id == MODEL_ID for binding in parity.bindings)
    ####
