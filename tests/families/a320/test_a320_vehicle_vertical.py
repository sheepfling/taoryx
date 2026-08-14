"""Focused vertical proof for the A320 plug-in's low-fidelity public contract."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path
from typing import Any, cast

import pytest
from taoryx_a320.resources import model_resource_root

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
from taoryx.vehicle_composition import load_vehicle_composition_request
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_parity_witnesses import (
    load_vehicle_execution_parity_witness_catalog,
    validate_vehicle_execution_parity_witnesses,
)
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog, validate_vehicle_execution_witnesses
from tools.extract_a320_plugin_assets import check as check_a320_plugin_assets
from tools.extract_a320_plugin_assets import extract as extract_a320_plugin_assets

MODEL_ID = "a320_openap_3dof"
MODEL_VERSION = "1.0.0+composition-v1"
PACKAGE_ID = "taoryx.a320"
PACKAGE_VERSION = "0.1.0a0"
PROVIDER_ID = "taoryx.a320.mission-composition"
MISSION_ID = "powered_fixed_wing_racetrack_v1"
ROOT = Path(__file__).resolve().parents[3]


def test_a320_discovery_defers_lqi_runtime_and_numerics() -> None:
    """Static native-coordinate metadata must not import the A320 LQI runtime."""

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(ROOT / "src"),
            str(ROOT / "packages/taoryx-a320/src"),
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
from taoryx_a320.plugin import PLUGIN


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
    "taoryx.a320_composition_episode",
    "taoryx.a320_mission_capability",
    "taoryx.a320_reduced_batch_episode_parity",
    "taoryx.a320_reduced_execution",
    "taoryx.control_allocation",
    "taoryx.family_adapter",
    "taoryx.family_adapter_registry",
    "taoryx.local_native_coordinate_lqi",
    "taoryx.local_native_coordinate_lqi_composition_execution",
    "taoryx.trajectory.a320_adapter",
    "taoryx.trajectory.a320_openap",
    "taoryx.trajectory.a320_pseudo6dof",
    "taoryx.trajectory.a320_racetrack",
)
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.a320", "taoryx_a320.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.a320",)
assert catalog.plugin_revision("taoryx.a320").version == "0.1.0a0"
assert not set(deferred) & set(sys.modules)
assert tuple(item.id for item in catalog.records("mission_capability_adapter")) == (
    "taoryx.a320_openap_racetrack.capability_scaled.v1",
    "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1",
)
assert all(isinstance(item.value, DeferredSemanticPreflightHandler) for item in catalog.records("semantic_preflight_handler"))
screens = catalog.build_local_native_coordinate_lqi_screen_registry()
selection = SimpleNamespace(
    family_id="a320_openap_3dof",
    mission="a320_local_native_coordinate_lqi_screen_v1",
    fidelity="pseudo_6dof",
    initialization=SimpleNamespace(id="pseudo6dof_cruise_attitude_local_point"),
)
definition = screens.resolve(selection)
assert definition is not None
assert definition.supports(selection)
advertisement = definition.public_advertisement()
assert advertisement["control_realization"] == "native_named_coordinates"
assert advertisement["native_controls"] == [
    {"id": "aileron_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
    {"id": "elevator_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
    {"id": "rudder_rad", "unit": "rad", "lower": -0.2, "upper": 0.2},
]
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
    """Discover only the A320 package for this fast vertical contract."""

    return discover_plugins(include_external=False, selected=(PACKAGE_ID,))
    ####


def _provider(plugins: PluginCatalog) -> Any:
    """Resolve the A320 provider through the selected package catalog."""

    return plugins.build_mission_composition_provider_registry().provider(PROVIDER_ID)
    ####


def _prepared_configuration(provider: Any) -> Any:
    """Prepare the package-owned pseudo-6DOF reduced composition witness."""

    request = load_vehicle_composition_request(model_resource_root() / "examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml")
    configuration = configuration_instance_from_vehicle_request(provider, request)
    return provider.validate_configuration(configuration)
    ####


def _forbid_catalog_rediscovery(monkeypatch: pytest.MonkeyPatch) -> None:
    """Fail if a focused public route tries to expand into an aggregate scan."""

    def unexpected_discovery(*args: object, **kwargs: object) -> object:
        del args, kwargs
        raise AssertionError("focused A320 execution unexpectedly rediscovered the aggregate plug-in catalog")
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


def test_a320_packaged_data_is_a_current_exact_extract(tmp_path) -> None:
    """The A320 wheel cannot retain stale or foreign exploded corpus assets."""

    assert check_a320_plugin_assets() == ()

    temporary_target = tmp_path / "a320-package-data"
    extract_a320_plugin_assets(temporary_target)
    corpus_root = temporary_target / "resources/aerospace/daveml/taoryx-corpus-v1.1"
    assert {path.relative_to(corpus_root).as_posix() for path in corpus_root.rglob("*") if path.is_file()} == {
        "NOTICE.md",
        "corpus.zip",
    }
    assert not (corpus_root / "qualified-models").exists()
    assert not (corpus_root / "source-corpora").exists()

    (temporary_target / "obsolete-a320-asset.txt").write_text("obsolete\n", encoding="utf-8")
    assert check_a320_plugin_assets(temporary_target) == ("unexpected generated A320 asset: obsolete-a320-asset.txt",)
    ####


def test_a320_selected_catalog_declares_and_resolves_its_own_data_fragment(plugins: PluginCatalog) -> None:
    """A focused A320 host must not silently read sibling catalog fragments."""

    contribution = plugins.contribution("vehicle_catalog_fragment", "taoryx.a320.vehicle-catalog")
    fragment = contribution.value
    assert isinstance(fragment, VehicleCatalogFragment)
    assert fragment.resource_package == "taoryx_a320"
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
        "a320-3dof-batch",
        "a320-pseudo6dof-batch",
        "a320-local-native-coordinate-lqi-screen-batch",
        "a320-3dof-episode",
        "a320-pseudo6dof-episode",
    }
    assert {item.id for item in witnesses.variant_witnesses} == {"a320-operating-mass-3dof"}
    assert {item.id for item in load_vehicle_execution_parity_witness_catalog(plugins=plugins).witnesses} == {
        "a320-3dof-kinematic-guidance",
        "a320-pseudo6dof-kinematic-guidance",
    }

    with plugin_catalog_scope(plugins):
        assert vehicle_catalog_resources("verification/vehicle_composition_registry.yaml") == resources
        assert {binding.family_id for binding in load_vehicle_execution_binding_catalog().bindings} == {MODEL_ID}
        assert {item.id for item in load_vehicle_execution_witness_catalog().witnesses} == {
            "a320-3dof-batch",
            "a320-pseudo6dof-batch",
            "a320-local-native-coordinate-lqi-screen-batch",
            "a320-3dof-episode",
            "a320-pseudo6dof-episode",
        }
        assert {item.id for item in load_vehicle_execution_parity_witness_catalog().witnesses} == {
            "a320-3dof-kinematic-guidance",
            "a320-pseudo6dof-kinematic-guidance",
        }

    with pytest.raises(FileNotFoundError, match="do not own resource"):
        vehicle_catalog_resources("verification/family_qualification_missions.yaml", plugins=plugins)
    ####


def test_a320_focused_provider_advertises_versioned_reduced_controls(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Discovery, model metadata, and authoring expose the shared low-tier API."""

    revision = plugins.plugin_revision(PACKAGE_ID)
    assert revision.package == "taoryx-a320"
    assert revision.version == PACKAGE_VERSION
    assert revision.api_version == "1"
    assert revision.contribution_count > 0
    assert len(revision.fingerprint) == 64

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


def test_a320_common_batch_route_retains_the_selected_plugin_catalog(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The common runner uses the A320 catalog without sibling discovery."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    response = build_registry_mission_composition_runner(provider).run(
        MissionCompositionRunRequest(
            request_id="a320-focused-vertical-batch",
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


def test_a320_common_session_route_retains_scope_and_can_switch_to_waypoint_guidance(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The session API exposes introspectable kinematic and waypoint controls."""

    provider = _provider(plugins)
    prepared = _prepared_configuration(provider)
    native_provider = provider.resolve_provider()
    assert native_provider.plugin_catalog is plugins
    _forbid_catalog_rediscovery(monkeypatch)

    manager = MissionCompositionSessionManager(provider)
    descriptor = manager.open(
        MissionCompositionOpenSessionRequest(
            session_id="a320-focused-vertical-session",
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            prepared_configuration=prepared,
            authority_profile_id="kinematic_guidance",
            seed=13,
            integration_step_s=0.02,
        )
    )

    assert descriptor.active_authority_profile_id == "kinematic_guidance"
    assert descriptor.action_schema_projection == "selected_semantic_profile"
    assert {item.id for item in descriptor.action_schema} == {
        "guidance.speed.command",
        "guidance.flight_path_angle.command",
        "guidance.heading.command",
        "guidance.bank.command",
    }

    stepped = manager.step(
        MissionCompositionSessionStepRequest(
            session_id=descriptor.session_id,
            action={"guidance.speed.command": 230.0},
            duration_s=0.02,
            expected_sequence=0,
        )
    )
    feedback = {item.channel_id: item for item in stepped.control_feedback}
    assert stepped.sequence == 1
    assert stepped.observation.lifecycle == "active"
    assert feedback["guidance.speed.command"].disposition == "applied_as_requested"
    assert feedback["guidance.speed.command"].achievement_status == "observed"

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
    }
    assert manager.close(MissionCompositionCloseSessionRequest(session_id=descriptor.session_id)).lifecycle == "closed"
    ####


def test_a320_reduced_witness_and_parity_gates_do_not_expand_to_other_plugins(
    plugins: PluginCatalog,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The quick gate covers reduced routes without the local LQI replay."""

    _forbid_catalog_rediscovery(monkeypatch)
    witnesses = validate_vehicle_execution_witnesses(
        witness_ids=(
            "a320-3dof-batch",
            "a320-pseudo6dof-batch",
            "a320-3dof-episode",
            "a320-pseudo6dof-episode",
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
