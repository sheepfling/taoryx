"""Focused vertical proof for the HL-20 local direct-wrench plug-in contract."""

from __future__ import annotations

from typing import Any, cast

import pytest
from taoryx_hl20.resources import model_resource_root

from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import (
    PluginCatalog,
    VehicleCatalogFragment,
    discover_plugins,
    plugin_catalog_scope,
)
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
from taoryx.vehicle_catalog_resources import vehicle_catalog_resource, vehicle_catalog_resources
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_parity_witnesses import load_vehicle_execution_parity_witness_catalog
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from tools.extract_hl20_plugin_assets import check as check_hl20_plugin_assets
from tools.extract_hl20_plugin_assets import extract as extract_hl20_plugin_assets

MODEL_ID = "hl20_mod_k"
MODEL_VERSION = "composition-registry-v1"
PACKAGE_ID = "taoryx.hl20"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.hl20.mission-composition"
MISSION_ID = "hl20_local_direct_wrench_screen_v1"
FIDELITY = "rigid_body_6dof_direct_wrench"
COMPOSITION = "hl20_local_direct_wrench_screen_compose.yaml"
REACHABILITY_MISSION_IDS = {
    "hl20_source_booster_release_replay_v1",
    "lifting_body_glide_energy_management_v1",
}


@pytest.fixture(scope="module")
def plugins() -> PluginCatalog:
    """Discover only the independently installable HL-20 plug-in."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the HL-20 provider from the caller's selected plug-in catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned local direct-wrench screen configuration."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / COMPOSITION)
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused HL-20 route expands into aggregate plug-in discovery."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused HL-20 execution unexpectedly rediscovered the aggregate plug-in catalog")
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


def test_hl20_packaged_data_is_a_current_exact_extract(tmp_path) -> None:
    """The HL-20 wheel cannot retain stale source/evidence assets."""

    assert check_hl20_plugin_assets() == ()

    temporary_target = tmp_path / "hl20-package-data"
    extract_hl20_plugin_assets(temporary_target)
    (temporary_target / "obsolete-hl20-asset.txt").write_text("obsolete\n", encoding="utf-8")

    assert check_hl20_plugin_assets(temporary_target) == ("unexpected generated HL-20 asset: obsolete-hl20-asset.txt",)
    ####


def test_hl20_catalog_and_optional_reachability_overlays_have_separate_owners(plugins: PluginCatalog) -> None:
    """Base HL-20 discovery must not advertise optional reachability routes."""

    contribution = plugins.contribution("vehicle_catalog_fragment", "taoryx.hl20.vehicle-catalog")
    fragment = contribution.value
    assert isinstance(fragment, VehicleCatalogFragment)
    assert fragment.resource_package == "taoryx_hl20"
    assert fragment.resource_root == "data"
    assert fragment.family_ids == (MODEL_ID,)

    base_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=plugins)
    assert base_resources == (model_resource_root() / "verification/vehicle_composition_registry.yaml",)
    assert all("taoryx_reachability" not in source.parts for source in base_resources)
    resolved = load_resolved_vehicle_composition_catalog(plugins=plugins).vehicle(MODEL_ID).declaration
    assert not REACHABILITY_MISSION_IDS & {item.id for item in resolved.mission_templates}
    assert not REACHABILITY_MISSION_IDS & {item.mission for item in load_vehicle_execution_binding_catalog(plugins=plugins).bindings}
    assert {item.id for item in load_vehicle_execution_witness_catalog(plugins=plugins).witnesses} == {
        "hl20-local-direct-wrench-screen-batch",
        "hl20-local-direct-wrench-screen-episode",
        "hl20-local-direct-wrench-lqi-screen-batch",
        "hl20-source-surface-pitch-authority-screen-batch",
        "hl20-source-surface-attitude-rate-lqi-screen-batch",
    }
    assert {item.id for item in load_vehicle_execution_parity_witness_catalog(plugins=plugins).witnesses} == {
        "hl20-local-direct-wrench",
    }
    with pytest.raises(FileNotFoundError, match="do not own resource"):
        vehicle_catalog_resource(
            "examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
            plugins=plugins,
        )

    with plugin_catalog_scope(plugins):
        assert vehicle_catalog_resources("verification/vehicle_composition_registry.yaml") == base_resources
        assert not REACHABILITY_MISSION_IDS & {
            item.id for item in load_resolved_vehicle_composition_catalog().vehicle(MODEL_ID).declaration.mission_templates
        }
        assert {binding.family_id for binding in load_vehicle_execution_binding_catalog().bindings} == {MODEL_ID}

    ####


def test_hl20_focused_provider_advertises_versioned_local_wrench_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Discovery exposes the local wrench boundary, not the reachability overlay."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-hl20"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64

    _forbid_catalog_rediscovery(monkeypatch)
    provider = _provider(plugins)
    model = provider.model(MODEL_ID)
    schema = provider.get_model_schema(MODEL_ID)

    assert tuple(item.id for item in provider.list_models()) == (MODEL_ID,)
    assert provider.metadata.version == PACKAGE_VERSION
    assert model.version == MODEL_VERSION
    assert schema.model_version == MODEL_VERSION
    assert model.output_schema.model_version == MODEL_VERSION
    assert len(model.metadata_fingerprint) == 64
    assert len(model.configuration_schema_fingerprint) == 64
    assert len(model.output_schema_fingerprint) == 64
    assert model.common_runner_operations == ("batch", "step")
    assert not REACHABILITY_MISSION_IDS & {item.id for item in model.mission_templates}

    plan = build_model_authoring_plan(
        plugins.build_mission_composition_provider_registry(),
        plugins.build_controller_tuning_campaign_registry(),
        PROVIDER_ID,
        MODEL_ID,
        family_adapters=plugins.build_family_adapter_registry(),
        fidelity=FIDELITY,
        realization_id=FIDELITY,
        mission_template_id=MISSION_ID,
    )

    assert plan["status"] == "ready_to_author"
    selection = cast(dict[str, object], plan["selection"])
    assert selection["model_version"] == MODEL_VERSION
    controller = cast(dict[str, object], plan["controller_automation"])
    assert controller["default_authority_id"] == "direct_wrench"
    assert {item["id"] for item in cast(list[dict[str, object]], controller["authorities"])} == {"direct_wrench"}
    channels = cast(list[dict[str, object]], controller["channels"])
    assert {item["id"] for item in channels} == {"wrench.force.command", "wrench.moment.command"}
    assert not any(str(item["id"]).startswith("effector.") for item in channels)
    ####


def test_hl20_composition_compiler_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Direct composition compilation keeps the caller's focused HL-20 scope."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition" / COMPOSITION)
    _forbid_catalog_rediscovery(monkeypatch)

    composition = compile_vehicle_composition(request, plugins=plugins)

    assert composition.family_id == MODEL_ID
    assert composition.fidelity == FIDELITY
    assert composition.mission == MISSION_ID
    ####


def test_hl20_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The common batch API stays within HL-20's selected plug-in catalog."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="hl20-focused-vertical-batch",
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
        (MODEL_ID, FIDELITY, FIDELITY),
    ]
    assert len(response.result.objects[0].samples) == 3
    ####


def test_hl20_common_session_route_retains_scope_and_direct_wrench_feedback(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The session API accepts the advertised direct-wrench control authority."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="hl20-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            authority_profile_id="direct_wrench",
            seed=13,
            integration_step_s=0.002,
        )
    )

    assert descriptor.active_authority_profile_id == "direct_wrench"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id for item in descriptor.action_schema} == {"wrench.force.command", "wrench.moment.command"}
    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={
                "wrench.force.command": [100_000.0, 0.0, 0.0],
                "wrench.moment.command": [0.0, 0.0, 0.0],
            },
            duration_s=0.004,
            expected_sequence=0,
        )
    )
    feedback = {item.channel_id: item for item in stepped.control_feedback}
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert feedback["wrench.force.command"].disposition == "applied_as_requested"
    assert feedback["wrench.moment.command"].disposition == "applied_as_requested"
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####
