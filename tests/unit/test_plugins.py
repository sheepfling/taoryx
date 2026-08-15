from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass
from filecmp import cmp
from pathlib import Path
from typing import NoReturn

import pytest
import yaml
from taoryx_dual_launch.plugin import PLUGIN as DUAL_LAUNCH_PLUGIN
from taoryx_simple_aero.plugin import PLUGIN as SIMPLE_AERO_PLUGIN

import taoryx.runtime.cli as runtime_cli
from taoryx.builtin_plugins import source_plugin_entry_points
from taoryx.composition_episode import registered_episode_factory_ids
from taoryx.local_controller_screen_advertisements import LocalControllerScreenAdvertisement
from taoryx.local_native_coordinate_lqi_screen_registry import LocalNativeCoordinateLqiScreenDefinition
from taoryx.plugins import (
    BATCH_FACTORY_REQUEST_CONTRACT,
    PluginCatalog,
    PluginCollisionError,
    PluginCompatibilityError,
    PluginContribution,
    PluginDefinition,
    PluginEntryPointDeclaration,
    PluginLoadError,
    PluginMetadata,
    PluginRegistrar,
    VehicleCatalogFragment,
    VehicleCatalogOverlayFragment,
    declared_plugin_entry_points,
    discover_plugins,
)
from taoryx.runtime.cli import main
from taoryx.trajectory.configuration_contract import ConfigurableTrajectoryProviderRegistry
from taoryx.vehicle_batch_execution import registered_vehicle_batch_factory_ids
from taoryx.vehicle_runtime_lowering import build_vehicle_runtime_adapter_registry


@dataclass(frozen=True)
class _EntryPoint:
    name: str
    value: str
    target: object
    distribution: str | None = None
    version: str | None = None

    def load(self) -> object:
        return self.target
        ####

    ####


def _model_plugin(
    plugin_id: str,
    model_id: str,
    *,
    api_version: str = "1",
    version: str = "1.0.0",
) -> PluginDefinition:
    def register(registrar: PluginRegistrar) -> None:
        registrar.register_model(model_id, object())
        ####

    return PluginDefinition(
        PluginMetadata(
            id=plugin_id,
            package=plugin_id,
            version=version,
            api_version=api_version,
            description=f"Test plug-in {plugin_id}.",
        ),
        register,
    )
    ####


def _unreachable_native_lqi_config() -> NoReturn:
    """Satisfy the static definition type in discovery-only registry tests."""

    raise AssertionError("the test registry must not construct a native LQI configuration")
    ####


def test_native_lqi_runtime_and_authoring_advertisements_cannot_drift() -> None:
    """One native screen has one package-owned executable and public contract."""

    metadata = PluginMetadata(
        id="example.native-lqi",
        package="example-native-lqi",
        version="1.0.0",
        api_version="1",
        description="Native LQI test plug-in.",
    )
    payload: dict[str, object] = {
        "schema": "taoryx.local-controller-screen-advertisement/v1alpha1",
        "id": "example-native-lqi-screen-v1",
        "mission_template_id": "example_native_lqi_screen_v1",
        "fidelity": "pseudo_6dof",
        "operations": ["batch"],
        "control_realization": "native_named_coordinates",
        "controller": {"method": "lqi"},
        "native_controls": [{"id": "roll_command", "unit": "rad", "lower": -0.1, "upper": 0.1}],
        "claim_boundary": "A test-only named-coordinate screen.",
    }
    definition = LocalNativeCoordinateLqiScreenDefinition(
        id="example-native-lqi-screen-v1",
        family_id="example_native",
        mission_id="example_native_lqi_screen_v1",
        fidelity="pseudo_6dof",
        initialization_id="example_initialization",
        segment_id="native_lqi",
        capability_adapter_id="example.native-lqi.capability.v1",
        config_factory=_unreachable_native_lqi_config,
        advertisement=payload,
        claim_boundary="A test-only named-coordinate screen.",
    )
    advertisement = LocalControllerScreenAdvertisement(
        id="example-native-lqi-screen-v1",
        provider_id="example.native-lqi.mission-composition",
        model_id="example_native",
        family_id="example_native",
        fidelity="pseudo_6dof",
        realization_id="example_pseudo_6dof",
        mission_template_id="example_native_lqi_screen_v1",
        advertisement=payload,
    )
    contributions = (
        PluginContribution("local_native_coordinate_lqi_screen_definition", definition.id, metadata, definition),
        PluginContribution("local_controller_screen_advertisement", advertisement.id, metadata, advertisement),
    )

    catalog = PluginCatalog(plugins=(metadata,), contributions=contributions, diagnostics=())
    assert catalog.build_local_native_coordinate_lqi_screen_registry().definitions == (definition,)

    mismatched = advertisement.model_copy(
        update={"advertisement": {**payload, "native_controls": []}},
    )
    with pytest.raises(ValueError, match="mismatched executable and advertised contract"):
        PluginCatalog(
            plugins=(metadata,),
            contributions=(
                contributions[0],
                PluginContribution("local_controller_screen_advertisement", mismatched.id, metadata, mismatched),
            ),
            diagnostics=(),
        )
    ####


def test_plugin_revision_is_scoped_to_its_owned_identity_and_contributions() -> None:
    """An unrelated package update must not invalidate a focused plug-in cache."""

    alpha = _model_plugin("example.alpha", "alpha-model")
    beta = _model_plugin("example.beta", "beta-model")
    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint("example.alpha", "example:alpha", alpha),
            _EntryPoint("example.beta", "example:beta", beta),
        ),
    )

    alpha_revision = catalog.plugin_revision("example.alpha")
    assert alpha_revision.version == "1.0.0"
    assert alpha_revision.contribution_count == 1
    assert alpha_revision.fingerprint == catalog.plugin_fingerprint("example.alpha")
    assert len(alpha_revision.fingerprint) == 64
    assert catalog.public_dict()["plugin_revisions"] == [item.public_dict() for item in catalog.plugin_revisions]

    unrelated_update = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint("example.alpha", "example:alpha", alpha),
            _EntryPoint("example.beta", "example:beta", _model_plugin("example.beta", "beta-model", version="2.0.0")),
        ),
    )
    assert unrelated_update.fingerprint != catalog.fingerprint
    assert unrelated_update.plugin_fingerprint("example.alpha") == alpha_revision.fingerprint

    augmented = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint("example.alpha", "example:alpha", alpha),
            _EntryPoint("example.beta", "example:beta", beta),
            _EntryPoint("example.gamma", "example:gamma", _model_plugin("example.gamma", "gamma-model")),
        ),
    )
    assert augmented.fingerprint != catalog.fingerprint
    assert augmented.plugin_fingerprint("example.alpha") == alpha_revision.fingerprint

    changed_alpha = discover_plugins(
        include_builtin=False,
        entry_points=(
            _EntryPoint("example.alpha", "example:alpha", _model_plugin("example.alpha", "alpha-model-v2")),
            _EntryPoint("example.beta", "example:beta", beta),
        ),
    )
    assert changed_alpha.plugin_fingerprint("example.alpha") != alpha_revision.fingerprint
    ####


def test_source_checkout_declarations_are_the_canonical_plugin_inventory() -> None:
    """Source fallback must derive identity and order from package metadata."""

    source_declarations = source_plugin_entry_points()
    declared = declared_plugin_entry_points(include_external=False)
    catalog = discover_plugins(include_external=False)

    assert tuple(item.name for item in source_declarations) == tuple(sorted(item.name for item in source_declarations))
    assert [item.public_dict() for item in declared] == [
        {
            "plugin_id": item.name,
            "target": item.value,
            "origin": "source_checkout",
            "distribution": item.distribution,
            "version": item.version,
        }
        for item in source_declarations
    ]
    assert tuple(item.id for item in catalog.plugins) == tuple(item.name for item in source_declarations)
    assert {
        item.id: (item.package, item.version)
        for item in catalog.plugins
    } == {
        item.name: (item.distribution, item.version)
        for item in source_declarations
    }
    ####


def test_entry_point_declaration_validation_happens_before_target_import() -> None:
    """A malformed declaration cannot execute code in either scan mode."""

    loads: list[str] = []

    class _NeverLoad:
        def __init__(self, name: str, value: str) -> None:
            self.name = name
            self.value = value
            ####

        def load(self) -> object:
            loads.append("load")
            raise AssertionError("invalid entry point was loaded")
            ####

    invalid_name = _NeverLoad("Example Invalid", "example:PLUGIN")
    with pytest.raises(PluginLoadError, match="not a valid plug-in ID"):
        discover_plugins(include_builtin=False, entry_points=(invalid_name,))
    assert loads == []

    catalog = discover_plugins(include_builtin=False, entry_points=(invalid_name,), strict=False)
    assert catalog.plugins == ()
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [("Example Invalid", "failed")]
    assert loads == []

    invalid_target = _NeverLoad("example.invalid-target", "example-plugin:PLUGIN")
    with pytest.raises(PluginLoadError, match="must use a valid 'module:attribute' target"):
        discover_plugins(include_builtin=False, entry_points=(invalid_target,))
    assert loads == []
    ####


def test_duplicate_entry_point_names_fail_closed_without_loading_either_target() -> None:
    """There is no first-entry-point-wins path for a shared plug-in ID."""

    loads: list[str] = []

    class _NeverLoad:
        def __init__(self, value: str) -> None:
            self.name = "example.duplicate"
            self.value = value
            ####

        def load(self) -> object:
            loads.append(self.value)
            raise AssertionError("ambiguous entry point was loaded")
            ####

    entry_points = (_NeverLoad("example:first"), _NeverLoad("example:second"))
    with pytest.raises(PluginLoadError, match="declared more than once"):
        discover_plugins(include_builtin=False, entry_points=entry_points)
    assert loads == []

    catalog = discover_plugins(include_builtin=False, entry_points=entry_points, strict=False)
    assert catalog.plugins == ()
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [("example.duplicate", "failed")]
    assert loads == []
    ####


def test_entry_point_order_is_deterministic_not_input_order() -> None:
    """Entry-point names establish stable plug-in and contribution ordering."""

    alpha = _model_plugin("example.alpha", "alpha-model")
    beta = _model_plugin("example.beta", "beta-model")
    ordered = (
        _EntryPoint("example.alpha", "example:alpha", alpha),
        _EntryPoint("example.beta", "example:beta", beta),
    )
    reversed_order = tuple(reversed(ordered))

    first = discover_plugins(include_builtin=False, entry_points=reversed_order)
    second = discover_plugins(include_builtin=False, entry_points=ordered)

    assert tuple(item.id for item in first.plugins) == ("example.alpha", "example.beta")
    assert tuple(item.id for item in first.records("model")) == ("alpha-model", "beta-model")
    assert first.fingerprint == second.fingerprint
    ####


def test_entry_point_distribution_and_version_must_match_plugin_metadata() -> None:
    """The entry point binds the distribution release to the loaded plug-in."""

    plugin = _model_plugin("example.metadata", "metadata-model")
    package_mismatch = _EntryPoint(
        "example.metadata",
        "example:metadata",
        plugin,
        distribution="other-plugin",
        version="1.0.0",
    )
    with pytest.raises(PluginLoadError, match="declares distribution"):
        discover_plugins(include_builtin=False, entry_points=(package_mismatch,))

    version_mismatch = _EntryPoint(
        "example.metadata",
        "example:metadata",
        plugin,
        distribution="example-metadata",
        version="2.0.0",
    )
    catalog = discover_plugins(include_builtin=False, entry_points=(version_mismatch,), strict=False)
    assert catalog.plugins == ()
    assert catalog.diagnostics[0].status == "failed"
    assert "declares distribution version" in catalog.diagnostics[0].message
    ####


def test_selected_plugin_scope_requires_a_declared_entry_point() -> None:
    """A host cannot silently select a misspelled or uninstalled plug-in."""

    with pytest.raises(PluginLoadError, match="has no available 'taoryx.plugins' declaration"):
        discover_plugins(include_builtin=False, include_external=False, selected=("example.missing",))

    catalog = discover_plugins(
        include_builtin=False,
        include_external=False,
        selected=("example.missing",),
        strict=False,
    )
    assert catalog.plugins == ()
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [("example.missing", "failed")]
    ####


def test_deferred_plugin_contributions_resolve_only_on_their_own_host_path() -> None:
    """Factory-shaped registrations keep discovery descriptive and cheap."""

    registrar = PluginRegistrar(
        PluginMetadata(
            id="example.lazy",
            package="example-lazy",
            version="1.0.0",
            api_version="1",
            description="A lazy contribution test plug-in.",
        )
    )
    factory_calls: list[str] = []

    @dataclass(frozen=True)
    class _FamilyRegistration:
        family_id: str

    @dataclass(frozen=True)
    class _ProviderMetadata:
        id: str

    @dataclass(frozen=True)
    class _TrimBinding:
        family_id: str

    class _Provider:
        metadata = _ProviderMetadata("example.lazy.provider")

    def build_family_registration() -> _FamilyRegistration:
        factory_calls.append("family")
        return _FamilyRegistration("example_lazy")
        ####

    def build_provider() -> _Provider:
        factory_calls.append("provider")
        return _Provider()
        ####

    def build_trim_binding() -> _TrimBinding:
        factory_calls.append("trim")
        return _TrimBinding("example_trim")
        ####

    def typed_batch_factory(request: object) -> object:
        return request
        ####

    family = registrar.register_family_adapter_factory("example_lazy", build_family_registration)
    registrar.register_model("example_lazy", family)
    registrar.register_mission_composition_provider_factory("example.lazy.provider", build_provider)
    trim = registrar.register_trim_evidence_binding_factory("example_trim", build_trim_binding)
    registrar.register_execution_factory_request_v1("example.lazy.batch.v1", typed_batch_factory)
    registrar.register_semantic_preflight_handler_callback("example.lazy.preflight.v1", lambda composition: composition)
    contributions = registrar.freeze()

    assert factory_calls == []
    assert family.family_id == "example_lazy"
    assert factory_calls == []
    assert family.resolve_registration().family_id == "example_lazy"
    provider = next(item.value for item in contributions if item.kind == "mission_composition_provider")
    registry = ConfigurableTrajectoryProviderRegistry((provider,))
    assert registry.provider("example.lazy.provider") is provider
    assert factory_calls == ["family"]
    assert provider.metadata.id == "example.lazy.provider"
    assert factory_calls == ["family", "provider"]
    assert trim.resolve_binding().family_id == "example_trim"
    assert factory_calls == ["family", "provider", "trim"]
    batch = next(item.value for item in contributions if item.kind == "execution_factory")
    assert getattr(batch, "__taoryx_batch_factory_contract__") == BATCH_FACTORY_REQUEST_CONTRACT
    preflight = next(item.value for item in contributions if item.kind == "semantic_preflight_handler")
    assert preflight.translator_id == "example.lazy.preflight.v1"
    assert preflight.handler(object()) is not None
    ####


def test_bundled_vehicle_registrations_cross_the_plugin_boundary_without_id_drift() -> None:
    catalog = discover_plugins(include_external=False)

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.a320",
        "taoryx.cadac",
        "taoryx.daveml",
        "taoryx.debug-models",
        "taoryx.dual-launch",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reachability",
        "taoryx.reference-models",
        "taoryx.simple-aero",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    )
    assert tuple(item.id for item in catalog.records("family_adapter")) == (
        "a320_openap_3dof",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "skywalker_x8",
        "b747",
        "x15",
    )
    assert {item.plugin.id for item in catalog.records("family_adapter")} == {
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    }
    assert tuple(item.id for item in catalog.records("model")) == tuple(item.id for item in catalog.records("family_adapter"))
    assert {item.plugin.id for item in catalog.records("model")} == {
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    }
    catalog_fragments = {
        item.plugin.id: item.value
        for item in catalog.records("vehicle_catalog_fragment")
    }
    assert set(catalog_fragments) == {
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reference-models",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    }
    assert all(isinstance(fragment, VehicleCatalogFragment) for fragment in catalog_fragments.values())
    assert {
        plugin_id: fragment.family_ids
        for plugin_id, fragment in catalog_fragments.items()
    } == {
        "taoryx.a320": ("a320_openap_3dof",),
        "taoryx.f16": ("f16_s119",),
        "taoryx.hl20": ("hl20_mod_k",),
        "taoryx.hummingbird": ("hummingbird",),
        "taoryx.nesc": ("reference_nesc_two_stage_rocket",),
        "taoryx.passive-bodies": ("tumbling_body",),
        "taoryx.reference-models": (),
        "taoryx.source-table-fixed-wing": ("skywalker_x8", "b747"),
        "taoryx.x15": ("x15",),
    }
    catalog_overlays = {item.id: item.value for item in catalog.records("vehicle_catalog_overlay_fragment")}
    assert set(catalog_overlays) == {
        "taoryx.reachability.x15-staged-catalog-overlay",
        "taoryx.reachability.hl20-catalog-overlay",
    }
    assert all(isinstance(fragment, VehicleCatalogOverlayFragment) for fragment in catalog_overlays.values())
    assert {
        identifier: (fragment.extends_fragment_ids, fragment.resource_root, fragment.family_ids)
        for identifier, fragment in catalog_overlays.items()
    } == {
        "taoryx.reachability.x15-staged-catalog-overlay": (
            ("taoryx.x15.vehicle-catalog",),
            "data/x15_overlay",
            ("x15",),
        ),
        "taoryx.reachability.hl20-catalog-overlay": (
            ("taoryx.hl20.vehicle-catalog",),
            "data/hl20_overlay",
            ("hl20_mod_k",),
        ),
    }
    assert tuple(item.id for item in catalog.records("trim_evidence_binding")) == (
        "reference_f16_s119",
        "reference_hl20_mod_k",
    )
    assert {item.plugin.id for item in catalog.records("trim_evidence_binding")} == {
        "taoryx.f16",
        "taoryx.hl20",
    }
    assert catalog.trim_evidence_binding("reference_f16_s119").family_id == "reference_f16_s119"
    assert catalog.trim_evidence_binding("reference_hl20_mod_k").family_id == "reference_hl20_mod_k"
    assert len(catalog.records("mission_capability_adapter")) == 33
    assert {item.plugin.id for item in catalog.records("mission_capability_adapter")} == {
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reachability",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    }
    assert sum(item.plugin.id == "taoryx.passive-bodies" for item in catalog.records("mission_capability_adapter")) == 1
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("mission_capability_adapter")) == 3
    assert sum(item.plugin.id == "taoryx.hummingbird" for item in catalog.records("mission_capability_adapter")) == 5
    assert sum(item.plugin.id == "taoryx.nesc" for item in catalog.records("mission_capability_adapter")) == 1
    assert sum(item.plugin.id == "taoryx.f16" for item in catalog.records("mission_capability_adapter")) == 6
    assert sum(item.plugin.id == "taoryx.a320" for item in catalog.records("mission_capability_adapter")) == 2
    assert sum(item.plugin.id == "taoryx.hl20" for item in catalog.records("mission_capability_adapter")) == 4
    assert sum(item.plugin.id == "taoryx.source-table-fixed-wing" for item in catalog.records("mission_capability_adapter")) == 7
    assert sum(item.plugin.id == "taoryx.x15" for item in catalog.records("mission_capability_adapter")) == 4
    assert registered_vehicle_batch_factory_ids(plugins=catalog) == tuple(sorted(item.id for item in catalog.records("execution_factory")))
    assert registered_episode_factory_ids(plugins=catalog) == tuple(sorted(item.id for item in catalog.records("episode_factory")))
    assert len(catalog.records("execution_factory")) == 29
    assert sum(item.plugin.id == "taoryx.a320" for item in catalog.records("execution_factory")) == 2
    assert sum(item.plugin.id == "taoryx.passive-bodies" for item in catalog.records("execution_factory")) == 1
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("execution_factory")) == 2
    assert sum(item.plugin.id == "taoryx.hummingbird" for item in catalog.records("execution_factory")) == 5
    assert sum(item.plugin.id == "taoryx.nesc" for item in catalog.records("execution_factory")) == 1
    assert sum(item.plugin.id == "taoryx.f16" for item in catalog.records("execution_factory")) == 6
    assert sum(item.plugin.id == "taoryx.hl20" for item in catalog.records("execution_factory")) == 3
    assert sum(item.plugin.id == "taoryx.source-table-fixed-wing" for item in catalog.records("execution_factory")) == 6
    assert sum(item.plugin.id == "taoryx.x15" for item in catalog.records("execution_factory")) == 3
    assert sum(item.plugin.id == "taoryx.a320" for item in catalog.records("semantic_preflight_handler")) == 2
    assert sum(item.plugin.id == "taoryx.passive-bodies" for item in catalog.records("semantic_preflight_handler")) == 1
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("semantic_preflight_handler")) == 3
    assert sum(item.plugin.id == "taoryx.hummingbird" for item in catalog.records("semantic_preflight_handler")) == 5
    assert sum(item.plugin.id == "taoryx.nesc" for item in catalog.records("semantic_preflight_handler")) == 1
    assert sum(item.plugin.id == "taoryx.f16" for item in catalog.records("semantic_preflight_handler")) == 6
    assert sum(item.plugin.id == "taoryx.hl20" for item in catalog.records("semantic_preflight_handler")) == 4
    assert sum(item.plugin.id == "taoryx.source-table-fixed-wing" for item in catalog.records("semantic_preflight_handler")) == 7
    assert sum(item.plugin.id == "taoryx.x15" for item in catalog.records("semantic_preflight_handler")) == 4
    assert len(catalog.records("episode_factory")) == 6
    assert len(catalog.records("batch_episode_parity_verifier")) == 6
    assert tuple(item.id for item in catalog.records("controller_tuning_campaign")) == (
        "a320-point-cruise-performance-lqr-v1",
        "a320-pseudo-cruise-attitude-v1",
        "f16-point-source-trim-translation-v1",
        "f16-pseudo-source-trim-attitude-v1",
        "f16-source-surface-local-lqr-v1",
        "f16-source-surface-schedule-lqr-v1",
        "f16-source-surface-local-lqi-v1",
        "f16-source-surface-schedule-lqi-v1",
        "hl20-source-subsonic-direct-wrench-v1",
        "hl20-source-subsonic-direct-wrench-lqi-v1",
        "hl20-source-surface-local-lqi-v1",
        "hummingbird-pseudo-hover-attitude-v1",
        "hummingbird-source-rotor-local-lqi-v1",
        "hummingbird-source-rotor-vertical-lqi-v1",
        "b747-source-surface-local-lqi-v1",
        "b747-language-backed-guidance-local-lqi-v1",
        "b747-language-backed-pseudo-guidance-local-lqi-v1",
        "x8-source-surface-local-lqi-v1",
        "x8-language-backed-guidance-local-lqi-v1",
        "x8-language-backed-pseudo-guidance-local-lqi-v1",
        "x15-source-release-direct-wrench-v1",
        "x15-source-release-direct-wrench-lqi-v1",
        "x15-source-surface-local-lqi-v1",
    )
    assert {item.plugin.id for item in catalog.records("controller_tuning_campaign")} == {
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    }
    assert tuple(item.id for item in catalog.records("local_controller_screen_advertisement")) == (
        "a320-pseudo-cruise-native-coordinate-lqi-screen-v1",
        "f16-source-trim-direct-wrench-lqr-screen-v1",
    )
    assert {item.plugin.id for item in catalog.records("local_controller_screen_advertisement")} == {
        "taoryx.a320",
        "taoryx.f16",
    }
    assert tuple(item.id for item in catalog.records("local_native_coordinate_lqi_screen_definition")) == (
        "a320-pseudo-cruise-native-coordinate-lqi-screen-v1",
    )
    assert {item.plugin.id for item in catalog.records("local_native_coordinate_lqi_screen_definition")} == {"taoryx.a320"}
    screens = catalog.build_local_controller_screen_advertisement_registry()
    assert [
        item.id
        for item in screens.matching(
            provider_id="taoryx.registry.mission-composition",
            model_id="f16_s119",
            family_id="f16_s119",
            fidelity="rigid_body_6dof_direct_wrench",
            realization_id="rigid_body_6dof_direct_wrench",
            mission_template_id="f16_local_physical_control_screen_v1",
        )
    ] == ["f16-source-trim-direct-wrench-lqr-screen-v1"]
    assert [
        item.id
        for item in screens.matching(
            provider_id="taoryx.a320.mission-composition",
            model_id="a320_openap_3dof",
            family_id="a320_openap_3dof",
            fidelity="pseudo_6dof",
            realization_id="jsbsim_surrogate_composite_pseudo6dof",
            mission_template_id="a320_local_native_coordinate_lqi_screen_v1",
        )
    ] == ["a320-pseudo-cruise-native-coordinate-lqi-screen-v1"]
    native_lqi_screens = catalog.build_local_native_coordinate_lqi_screen_registry()
    assert [item.id for item in native_lqi_screens.definitions] == ["a320-pseudo-cruise-native-coordinate-lqi-screen-v1"]
    direct_wrench_screens = catalog.build_local_direct_wrench_screen_registry()
    assert [item.id for item in direct_wrench_screens.definitions] == [
        "hl20-source-mach0p5-local-direct-wrench-v1",
        "hl20-source-subsonic-local-direct-wrench-lqi-v1",
        "hummingbird-source-hover-local-direct-wrench-v1",
        "x15-source-release-glide-local-direct-wrench-v1",
        "x15-source-release-glide-local-direct-wrench-lqi-v1",
    ]
    assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
        "taoryx.a320.mission-composition",
        "cadac",
        "taoryx.reference.mission-composition",
        "taoryx.debug.mission-composition-contract-probe",
        "taoryx.dual-launch.mission-composition",
        "taoryx.f16.mission-composition",
        "taoryx.hl20.mission-composition",
        "taoryx.hummingbird.mission-composition",
        "taoryx.nesc.mission-composition",
        "taoryx.passive-bodies.mission-composition",
        "taoryx.registry.mission-composition",
        "taoryx.simple-aero.mission-composition",
        "taoryx.x8.mission-composition",
        "taoryx.b747.mission-composition",
        "taoryx.x15.mission-composition",
    )
    composer_providers = catalog.build_mission_composition_provider_registry()
    assert tuple(item.metadata.id for item in composer_providers.providers) == (
        "taoryx.a320.mission-composition",
        "cadac",
        "taoryx.reference.mission-composition",
        "taoryx.debug.mission-composition-contract-probe",
        "taoryx.dual-launch.mission-composition",
        "taoryx.f16.mission-composition",
        "taoryx.hl20.mission-composition",
        "taoryx.hummingbird.mission-composition",
        "taoryx.nesc.mission-composition",
        "taoryx.passive-bodies.mission-composition",
        "taoryx.registry.mission-composition",
        "taoryx.simple-aero.mission-composition",
        "taoryx.x8.mission-composition",
        "taoryx.b747.mission-composition",
        "taoryx.x15.mission-composition",
    )
    assert tuple(item.id for item in catalog.records("mission_workflow_endpoint_catalog")) == (
        "taoryx.debug-models.workflow-endpoints",
        "taoryx.dual-launch.workflow-endpoints",
        "taoryx.simple-aero.workflow-endpoints",
    )
    assert len(composer_providers.list_models("taoryx.registry.mission-composition")) == 11
    assert composer_providers.model("taoryx.registry.mission-composition", "f16_s119").id == "f16_s119"
    aggregate = getattr(
        composer_providers.provider("taoryx.registry.mission-composition"),
        "resolve_provider",
    )()
    assert getattr(aggregate, "_plugin_catalog") is catalog
    assert composer_providers.public_dict()["schema"] == "taoryx.mission-composition-provider-catalog/v1"
    assert tuple(item.id for item in catalog.records("reachability_provider")) == ("taoryx.reachability.workbench",)
    reachability = catalog.build_reachability_provider_registry().provider("taoryx.reachability.workbench")
    assert reachability.metadata.families == ("hl20_mod_k", "x15")
    assert tuple(item.id for item in catalog.records("deployment_child_runtime")) == ("taoryx.passive-bodies.local-atmosphere-release.v1",)

    registry = build_vehicle_runtime_adapter_registry(plugins=catalog)
    assert tuple(item.family_id for item in registry.registrations) == tuple(item.id for item in catalog.records("family_adapter"))
    ####


def test_external_simple_aero_entry_point_registers_the_extracted_provider() -> None:
    entry_point = _EntryPoint(
        "taoryx.simple-aero",
        "taoryx_simple_aero.plugin:PLUGIN",
        SIMPLE_AERO_PLUGIN,
    )

    first = discover_plugins(include_builtin=False, entry_points=(entry_point,))
    second = discover_plugins(include_builtin=False, entry_points=(entry_point,))

    assert first.fingerprint == second.fingerprint
    contribution = first.contribution("trajectory_provider", "reference.point_mass")
    assert contribution.plugin.package == "taoryx-simple-aero"
    assert contribution.value.capabilities.families == ("simple_aero",)  # type: ignore[attr-defined]
    assert contribution.value.__class__.__module__ == "taoryx_simple_aero.provider"
    assert tuple(item.id for item in first.records("mission_composition_provider")) == (
        "taoryx.simple-aero.mission-composition",
    )
    assert tuple(item.id for item in first.records("mission_workflow_endpoint_catalog")) == (
        "taoryx.simple-aero.workflow-endpoints",
    )
    providers = first.build_trajectory_provider_registry()
    assert providers.provider("reference.point_mass") is contribution.value
    ####


def test_external_dual_launch_entry_point_registers_only_its_focused_workflow() -> None:
    """The standalone workflow has an installable entry-point contract."""

    entry_point = _EntryPoint(
        "taoryx.dual-launch",
        "taoryx_dual_launch.plugin:PLUGIN",
        DUAL_LAUNCH_PLUGIN,
    )

    first = discover_plugins(include_builtin=False, entry_points=(entry_point,))
    second = discover_plugins(include_builtin=False, entry_points=(entry_point,))

    assert first.fingerprint == second.fingerprint
    contribution = first.contribution("mission_composition_provider", "taoryx.dual-launch.mission-composition")
    assert contribution.plugin.package == "taoryx-dual-launch"
    assert tuple(item.id for item in first.records("mission_composition_provider")) == (
        "taoryx.dual-launch.mission-composition",
    )
    assert tuple(item.id for item in first.records("mission_workflow_endpoint_catalog")) == (
        "taoryx.dual-launch.workflow-endpoints",
    )
    provider = first.build_mission_composition_provider_registry().provider("taoryx.dual-launch.mission-composition")
    assert provider.metadata.version == "0.1.0a0"
    assert tuple(item.id for item in provider.list_models()) == ("dual_launch_glider",)
    assert provider.resolve_provider().__class__.__module__ == "taoryx_dual_launch.mission_composition"
    ####


def test_duplicate_typed_identity_fails_closed_and_non_strict_scan_is_atomic() -> None:
    first = _model_plugin("example.first", "shared-model")
    second = _model_plugin("example.second", "shared-model")
    entry_points = (
        _EntryPoint(first.metadata.id, "example:first", first),
        _EntryPoint(second.metadata.id, "example:second", second),
    )

    with pytest.raises(PluginCollisionError, match="shared-model"):
        discover_plugins(include_builtin=False, entry_points=entry_points)

    catalog = discover_plugins(include_builtin=False, entry_points=entry_points, strict=False)
    assert tuple(item.id for item in catalog.plugins) == ("example.first",)
    assert tuple(item.id for item in catalog.records("model")) == ("shared-model",)
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [
        ("example.first", "loaded"),
        ("example.second", "failed"),
    ]
    ####


def test_callback_failure_discards_every_staged_contribution() -> None:
    metadata = PluginMetadata(
        id="example.partial",
        package="example-partial",
        version="1.0.0",
        api_version="1",
        description="A callback-failure witness.",
    )

    def register(registrar: PluginRegistrar) -> None:
        registrar.register_model("must-not-leak", object())
        raise RuntimeError("registration stopped")
        ####

    plugin = PluginDefinition(metadata, register)
    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(_EntryPoint(metadata.id, "example:partial", plugin),),
        strict=False,
    )

    assert catalog.plugins == ()
    assert catalog.contributions == ()
    assert catalog.diagnostics[0].status == "failed"
    assert "registration stopped" in catalog.diagnostics[0].message
    ####


def test_incompatible_plugin_is_rejected_or_reported_without_contributions() -> None:
    plugin = _model_plugin("example.future", "future-model", api_version="999")
    entry_point = _EntryPoint(plugin.metadata.id, "example:future", plugin)

    with pytest.raises(PluginCompatibilityError, match="host API"):
        discover_plugins(include_builtin=False, entry_points=(entry_point,))

    catalog = discover_plugins(include_builtin=False, entry_points=(entry_point,), strict=False)
    assert catalog.plugins == ()
    assert catalog.contributions == ()
    assert catalog.diagnostics[0].status == "incompatible"
    ####


def test_disabled_plugin_is_not_loaded() -> None:
    class _MustNotLoadEntryPoint(_EntryPoint):
        def load(self) -> object:
            raise AssertionError("disabled entry point was loaded")
            ####

    entry_point = _MustNotLoadEntryPoint("example.disabled", "example:disabled", object())
    catalog = discover_plugins(
        include_builtin=False,
        entry_points=(entry_point,),
        disabled=("example.disabled",),
    )

    assert catalog.plugins == ()
    assert catalog.diagnostics[0].status == "disabled"
    ####


def test_plugin_cli_lists_typed_contributions_without_constructing_plants(capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["plugins", "list", "--no-external", "--json"]) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.plugin-catalog/v1"
    assert [item["id"] for item in payload["plugins"]] == [
        "taoryx.a320",
        "taoryx.cadac",
        "taoryx.daveml",
        "taoryx.debug-models",
        "taoryx.dual-launch",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reachability",
        "taoryx.reference-models",
        "taoryx.simple-aero",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    ]
    assert {item["kind"] for item in payload["contributions"]} == {
        "batch_episode_parity_verifier",
        "controller_tuning_campaign",
        "deployment_child_runtime",
        "episode_factory",
        "execution_factory",
        "family_adapter",
        "local_controller_screen_advertisement",
        "local_direct_wrench_screen_definition",
        "local_native_coordinate_lqi_screen_definition",
        "mission_capability_adapter",
        "mission_composition_provider",
        "mission_workflow_endpoint_catalog",
        "model",
        "model_format",
        "reachability_provider",
        "semantic_preflight_handler",
        "trajectory_provider",
        "trim_evidence_binding",
        "vehicle_catalog_fragment",
        "vehicle_catalog_overlay_fragment",
        "vehicle_interface_extension",
    }
    assert [item["plugin_id"] for item in payload["plugin_revisions"]] == [item["id"] for item in payload["plugins"]]
    assert all(len(item["fingerprint"]) == 64 for item in payload["plugin_revisions"])
    ####


def test_plugin_entry_point_cli_lists_source_declarations_without_loading_targets() -> None:
    """The declaration inventory works with only the core source path active."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(root / "src")
    script = """
import sys
from taoryx.runtime.cli import main

assert "taoryx_cadac.plugin" not in sys.modules
exit_code = main(["plugins", "entry-points", "--no-external", "--json"])
assert "taoryx_cadac.plugin" not in sys.modules
raise SystemExit(exit_code)
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
    payload = json.loads(completed.stdout)
    assert payload["schema"] == "taoryx.plugin-entry-point-catalog/v1"
    assert [item["plugin_id"] for item in payload["entry_points"]] == sorted(
        item["plugin_id"] for item in payload["entry_points"]
    )
    cadac = next(item for item in payload["entry_points"] if item["plugin_id"] == "taoryx.cadac")
    assert cadac == {
        "plugin_id": "taoryx.cadac",
        "target": "taoryx_cadac.plugin:PLUGIN",
        "origin": "source_checkout",
        "distribution": "taoryx-cadac",
        "version": "0.1.0a0",
    }
    ####


def test_plugin_cli_checks_full_installed_distribution_profile(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    catalog = discover_plugins(include_external=False)

    def discover_installed(**kwargs: object) -> object:
        assert kwargs == {"include_builtin": False, "strict": False}
        return catalog
        ####

    def declared_installed(**kwargs: object) -> tuple[PluginEntryPointDeclaration, ...]:
        assert kwargs == {"include_builtin": False}
        return tuple(
            PluginEntryPointDeclaration(
                plugin_id=plugin_id,
                target=f"example_{index}.plugin:PLUGIN",
                origin="installed",
                distribution=runtime_cli._OFFICIAL_PLUGIN_DISTRIBUTIONS[plugin_id],
                version="0.1.0a0",
            )
            for index, plugin_id in enumerate(runtime_cli._PLUGIN_INSTALL_PROFILES["full"])
        )
        ####

    monkeypatch.setattr(runtime_cli, "discover_plugins", discover_installed)
    monkeypatch.setattr(runtime_cli, "declared_plugin_entry_points", declared_installed)
    monkeypatch.setattr(runtime_cli.importlib_metadata, "version", lambda _name: "0.1.0a0")

    assert runtime_cli.main(["plugins", "check", "--profile", "full", "--json"]) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.plugin-installation-check/v1"
    assert payload["ready"] is True
    assert [item["distribution"] for item in payload["distributions"]] == [
        "taoryx",
        "taoryx-daveml",
        "taoryx-debug-models",
        "taoryx-a320",
        "taoryx-f16",
        "taoryx-hummingbird",
        "taoryx-nesc",
        "taoryx-passive-bodies",
        "taoryx-simple-aero",
        "taoryx-x15",
        "taoryx-hl20",
        "taoryx-source-table-fixed-wing",
        "taoryx-dual-launch",
        "taoryx-reference-models",
        "taoryx-reachability",
    ]
    assert all(item["loaded"] for item in payload["plugins"])
    assert all(item["entry_point_declared"] for item in payload["plugins"])
    assert all(item["entry_point_distribution_matches"] for item in payload["plugins"])
    assert all(item["entry_point_version_matches"] for item in payload["plugins"])
    ####


def test_plugin_cli_fails_closed_for_missing_model_profile_distribution(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    empty_catalog = discover_plugins(include_builtin=False, include_external=False)

    monkeypatch.setattr(runtime_cli, "discover_plugins", lambda **_kwargs: empty_catalog)
    monkeypatch.setattr(runtime_cli, "declared_plugin_entry_points", lambda **_kwargs: ())

    def installed_version(distribution: str) -> str:
        if distribution == "taoryx":
            return "0.1.0a0"
        raise runtime_cli.importlib_metadata.PackageNotFoundError(distribution)
        ####

    monkeypatch.setattr(runtime_cli.importlib_metadata, "version", installed_version)

    assert runtime_cli.main(["plugins", "check", "--profile", "models", "--json"]) == 2

    payload = json.loads(capsys.readouterr().out)
    assert payload["ready"] is False
    assert [item["distribution"] for item in payload["distributions"] if not item["installed"]] == [
        "taoryx-daveml",
        "taoryx-debug-models",
        "taoryx-a320",
        "taoryx-f16",
        "taoryx-hummingbird",
        "taoryx-nesc",
        "taoryx-passive-bodies",
        "taoryx-simple-aero",
        "taoryx-x15",
        "taoryx-hl20",
        "taoryx-source-table-fixed-wing",
        "taoryx-dual-launch",
    ]
    assert all(not item["loaded"] for item in payload["plugins"])
    assert all(not item["entry_point_declared"] for item in payload["plugins"])
    ####


def test_direct_developer_profile_excludes_the_compatibility_aggregate() -> None:
    """Make the installed developer route independent of legacy aggregation."""

    models = runtime_cli._PLUGIN_INSTALL_PROFILES["models"]
    developer = runtime_cli._PLUGIN_INSTALL_PROFILES["developer"]
    compatibility = runtime_cli._PLUGIN_INSTALL_PROFILES["compatibility"]
    full = runtime_cli._PLUGIN_INSTALL_PROFILES["full"]

    assert "taoryx.reference-models" not in models
    assert "taoryx.reference-models" not in developer
    assert compatibility == (
        "taoryx.daveml",
        "taoryx.a320",
        "taoryx.f16",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.simple-aero",
        "taoryx.x15",
        "taoryx.hl20",
        "taoryx.source-table-fixed-wing",
        "taoryx.dual-launch",
        "taoryx.reference-models",
    )
    assert "taoryx.debug-models" not in compatibility
    assert "taoryx.passive-bodies" not in compatibility
    assert "taoryx.reachability" not in compatibility
    assert developer == (*models, "taoryx.reachability")
    assert set(full) == {*developer, *compatibility}
    ####


def test_minimal_compatibility_aggregate_excludes_the_optional_passive_body() -> None:
    """The aggregate profile contains only its declared dependency closure."""

    catalog = discover_plugins(
        include_external=False,
        selected=runtime_cli._PLUGIN_INSTALL_PROFILES["compatibility"],
    )
    provider = catalog.build_mission_composition_provider_registry().provider(
        "taoryx.registry.mission-composition"
    )

    assert tuple(model.id for model in provider.list_models()) == (
        "a320_openap_3dof",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "skywalker_x8",
        "b747",
        "x15",
        "simple_aero",
        "dual_launch_glider",
    )
    ####


@pytest.mark.parametrize(
    ("plugin_id", "provider_ids"),
    (
        ("taoryx.a320", ("taoryx.a320.mission-composition",)),
        ("taoryx.f16", ("taoryx.f16.mission-composition",)),
        ("taoryx.hummingbird", ("taoryx.hummingbird.mission-composition",)),
        ("taoryx.nesc", ("taoryx.nesc.mission-composition",)),
        ("taoryx.passive-bodies", ("taoryx.passive-bodies.mission-composition",)),
        ("taoryx.x15", ("taoryx.x15.mission-composition",)),
        ("taoryx.hl20", ("taoryx.hl20.mission-composition",)),
        (
            "taoryx.source-table-fixed-wing",
            ("taoryx.x8.mission-composition", "taoryx.b747.mission-composition"),
        ),
    ),
)
def test_direct_catalog_provider_keeps_its_exact_selected_plugin_scope(
    plugin_id: str,
    provider_ids: tuple[str, ...],
) -> None:
    """Every catalog-scoped family host stays isolated from the aggregate."""

    catalog = discover_plugins(include_external=False, selected=(plugin_id,))
    providers = catalog.build_mission_composition_provider_registry()
    for provider_id in provider_ids:
        deferred_provider = providers.provider(provider_id)
        concrete_provider = getattr(deferred_provider, "resolve_provider")()
        assert getattr(concrete_provider, "_plugin_catalog") is catalog
    ####


def test_disabling_reference_models_removes_only_the_compatibility_aggregate() -> None:
    catalog = discover_plugins(
        include_external=False,
        disabled=("taoryx.reference-models",),
    )

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.a320",
        "taoryx.cadac",
        "taoryx.daveml",
        "taoryx.debug-models",
        "taoryx.dual-launch",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reachability",
        "taoryx.simple-aero",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    )
    assert tuple(item.id for item in catalog.records("family_adapter")) == (
        "a320_openap_3dof",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "skywalker_x8",
        "b747",
        "x15",
    )
    assert tuple(item.id for item in catalog.records("model")) == (
        "a320_openap_3dof",
        "f16_s119",
        "hl20_mod_k",
        "hummingbird",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "skywalker_x8",
        "b747",
        "x15",
    )
    assert len(catalog.records("mission_capability_adapter")) == 33
    assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
        "taoryx.a320.mission-composition",
        "cadac",
        "taoryx.reference.mission-composition",
        "taoryx.debug.mission-composition-contract-probe",
        "taoryx.dual-launch.mission-composition",
        "taoryx.f16.mission-composition",
        "taoryx.hl20.mission-composition",
        "taoryx.hummingbird.mission-composition",
        "taoryx.nesc.mission-composition",
        "taoryx.passive-bodies.mission-composition",
        "taoryx.simple-aero.mission-composition",
        "taoryx.x8.mission-composition",
        "taoryx.b747.mission-composition",
        "taoryx.x15.mission-composition",
    )
    assert tuple(item.id for item in catalog.records("mission_workflow_endpoint_catalog")) == (
        "taoryx.debug-models.workflow-endpoints",
        "taoryx.dual-launch.workflow-endpoints",
        "taoryx.simple-aero.workflow-endpoints",
    )
    assert len(catalog.records("semantic_preflight_handler")) == 33
    assert len(catalog.records("execution_factory")) == 29
    assert len(catalog.records("episode_factory")) == 6
    assert len(catalog.records("batch_episode_parity_verifier")) == 6
    assert len(catalog.records("controller_tuning_campaign")) == 23
    assert len(catalog.records("local_controller_screen_advertisement")) == 2
    assert len(catalog.records("local_native_coordinate_lqi_screen_definition")) == 1
    assert len(catalog.records("local_direct_wrench_screen_definition")) == 5
    assert tuple(item.id for item in catalog.records("vehicle_interface_extension")) == (
        "f16_s119",
        "hummingbird",
        "tumbling_body",
    )
    assert tuple(item.plugin.id for item in catalog.records("vehicle_interface_extension")) == (
        "taoryx.f16",
        "taoryx.hummingbird",
        "taoryx.passive-bodies",
    )
    assert tuple(item.id for item in catalog.records("trim_evidence_binding")) == (
        "reference_f16_s119",
        "reference_hl20_mod_k",
    )
    assert tuple(item.id for item in catalog.records("deployment_child_runtime")) == ("taoryx.passive-bodies.local-atmosphere-release.v1",)
    assert tuple(item.id for item in catalog.records("model_format")) == ("daveml",)
    assert tuple(item.id for item in catalog.records("trajectory_provider")) == ("reference.point_mass",)
    assert tuple(item.id for item in catalog.records("reachability_provider")) == ("taoryx.reachability.workbench",)
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [
        ("taoryx.a320", "loaded"),
        ("taoryx.cadac", "loaded"),
        ("taoryx.daveml", "loaded"),
        ("taoryx.debug-models", "loaded"),
        ("taoryx.dual-launch", "loaded"),
        ("taoryx.f16", "loaded"),
        ("taoryx.hl20", "loaded"),
        ("taoryx.hummingbird", "loaded"),
        ("taoryx.nesc", "loaded"),
        ("taoryx.passive-bodies", "loaded"),
        ("taoryx.reachability", "loaded"),
        ("taoryx.reference-models", "disabled"),
        ("taoryx.simple-aero", "loaded"),
        ("taoryx.source-table-fixed-wing", "loaded"),
        ("taoryx.x15", "loaded"),
    ]
    ####


def test_disabling_reachability_removes_only_reachability_owned_overlays() -> None:
    catalog = discover_plugins(
        include_external=False,
        disabled=("taoryx.reachability",),
    )

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.a320",
        "taoryx.cadac",
        "taoryx.daveml",
        "taoryx.debug-models",
        "taoryx.dual-launch",
        "taoryx.f16",
        "taoryx.hl20",
        "taoryx.hummingbird",
        "taoryx.nesc",
        "taoryx.passive-bodies",
        "taoryx.reference-models",
        "taoryx.simple-aero",
        "taoryx.source-table-fixed-wing",
        "taoryx.x15",
    )
    assert len(catalog.records("family_adapter")) == 9
    assert len(catalog.records("model")) == 9
    assert len(catalog.records("mission_capability_adapter")) == 30
    assert len(catalog.records("semantic_preflight_handler")) == 30
    assert len(catalog.records("execution_factory")) == 27
    assert len(catalog.records("episode_factory")) == 6
    assert len(catalog.records("batch_episode_parity_verifier")) == 6
    assert len(catalog.records("controller_tuning_campaign")) == 23
    assert len(catalog.records("local_controller_screen_advertisement")) == 2
    assert len(catalog.records("local_native_coordinate_lqi_screen_definition")) == 1
    assert len(catalog.records("local_direct_wrench_screen_definition")) == 5
    assert tuple(item.id for item in catalog.records("deployment_child_runtime")) == ("taoryx.passive-bodies.local-atmosphere-release.v1",)
    assert catalog.records("reachability_provider") == ()
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [
        ("taoryx.a320", "loaded"),
        ("taoryx.cadac", "loaded"),
        ("taoryx.daveml", "loaded"),
        ("taoryx.debug-models", "loaded"),
        ("taoryx.dual-launch", "loaded"),
        ("taoryx.f16", "loaded"),
        ("taoryx.hl20", "loaded"),
        ("taoryx.hummingbird", "loaded"),
        ("taoryx.nesc", "loaded"),
        ("taoryx.passive-bodies", "loaded"),
        ("taoryx.reachability", "disabled"),
        ("taoryx.reference-models", "loaded"),
        ("taoryx.simple-aero", "loaded"),
        ("taoryx.source-table-fixed-wing", "loaded"),
        ("taoryx.x15", "loaded"),
    ]
    ####


def test_core_source_tree_imports_without_any_model_distribution() -> None:
    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(root / "src")
    script = """
import json
import io
import sys
from contextlib import redirect_stdout
from pathlib import Path

# Editable plug-ins add their source roots through .pth files. Remove those
# paths so this subprocess still proves the core-only wheel boundary after the
# contributor environment has installed the full suite.
sys.path[:] = [
    entry
    for entry in sys.path
    if not any(part.startswith("taoryx-") for part in Path(entry).parts)
]
import taoryx
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins
from taoryx.runtime.cli import main

# The subprocess models an environment where the optional distributions are
# absent, so hide their metadata as well as their editable source paths.
plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False)
assert catalog.plugins == ()
assert catalog.contributions == ()
reachability_output = io.StringIO()
with redirect_stdout(reachability_output):
    assert main(["reachability", "list", "profiles"]) == 2
assert "install taoryx-reachability" in reachability_output.getvalue()
assert main(["plugins", "list", "--no-external", "--json"]) == 0
for forbidden in (
    "taoryx.x15_adapter",
    "taoryx.hl20_adapter",
    "taoryx.trajectory.a320_openap",
    "taoryx.trajectory.daveml_import",
    "taoryx.hummingbird_composition_episode",
):
    assert forbidden not in sys.modules
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
    payload = json.loads(completed.stdout)
    assert payload["plugins"] == []
    assert payload["contributions"] == []
    ####


def test_reference_model_source_tree_imports_without_reachability_distribution() -> None:
    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-debug-models/src",
            "packages/taoryx-a320/src",
            "packages/taoryx-f16/src",
            "packages/taoryx-hummingbird/src",
            "packages/taoryx-nesc/src",
            "packages/taoryx-passive-bodies/src",
            "packages/taoryx-simple-aero/src",
            "packages/taoryx-dual-launch/src",
            "packages/taoryx-x15/src",
            "packages/taoryx-hl20/src",
            "packages/taoryx-source-table-fixed-wing/src",
            "packages/taoryx-reference-models/src",
        )
    )
    script = """
import sys
from pathlib import Path

# Preserve the requested editable source roots while excluding the
# reachability/CADAC .pth entries installed by a full contributor bootstrap.
sys.path[:] = [
    entry
    for entry in sys.path
    if not any(part in {"taoryx-reachability", "taoryx-cadac"} for part in Path(entry).parts)
]
from taoryx.plugins import discover_plugins

catalog = discover_plugins(include_external=False)
assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.a320",
        "taoryx.daveml",
        "taoryx.debug-models",
        "taoryx.dual-launch",
        "taoryx.f16",
    "taoryx.hl20",
    "taoryx.hummingbird",
    "taoryx.nesc",
    "taoryx.passive-bodies",
    "taoryx.reference-models",
    "taoryx.simple-aero",
    "taoryx.source-table-fixed-wing",
    "taoryx.x15",
)
assert len(catalog.records("family_adapter")) == 9
assert len(catalog.records("mission_capability_adapter")) == 30
assert len(catalog.records("semantic_preflight_handler")) == 30
assert len(catalog.records("execution_factory")) == 27
assert len(catalog.records("local_controller_screen_advertisement")) == 2
assert len(catalog.records("local_native_coordinate_lqi_screen_definition")) == 1
assert len(catalog.records("local_direct_wrench_screen_definition")) == 5
assert catalog.records("reachability_provider") == ()
assert "taoryx.reachability_envelope" not in sys.modules
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


def test_reference_completion_audit_does_not_require_debug_models() -> None:
    """The compatibility package must not import development providers directly."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-a320/src",
            "packages/taoryx-f16/src",
            "packages/taoryx-hummingbird/src",
            "packages/taoryx-nesc/src",
            "packages/taoryx-passive-bodies/src",
            "packages/taoryx-simple-aero/src",
            "packages/taoryx-x15/src",
            "packages/taoryx-hl20/src",
            "packages/taoryx-source-table-fixed-wing/src",
            "packages/taoryx-reference-models/src",
        )
    )
    script = """
import sys
from pathlib import Path

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {"taoryx-debug-models", "taoryx-reachability", "taoryx-cadac"}
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.mission_composition_completion import build_mission_composition_completion_report

plugin_discovery._installed_entry_points = lambda: ()
report = build_mission_composition_completion_report()
assert report.status == "pass", report.diagnostics
assert "taoryx_debug_models" not in sys.modules
assert "taoryx.trajectory.contract_probe_mission_composition" not in sys.modules
assert "taoryx.trajectory.reference_mission_composition" not in sys.modules
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


def test_debug_provider_source_tree_imports_without_vehicle_distributions() -> None:
    """Development providers remain installable without physical model packages."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-debug-models/src",
        )
    )
    script = """
import sys
from pathlib import Path

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-daveml",
            "taoryx-a320",
            "taoryx-hummingbird",
            "taoryx-f16",
            "taoryx-nesc",
            "taoryx-passive-bodies",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-simple-aero",
            "taoryx-dual-launch",
            "taoryx-x15",
            "taoryx-hl20",
                "taoryx-source-table-fixed-wing",
                "taoryx-cadac",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.debug-models",)
assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
    "taoryx.reference.mission-composition",
    "taoryx.debug.mission-composition-contract-probe",
)
assert tuple(item.id for item in catalog.records("mission_workflow_endpoint_catalog")) == (
    "taoryx.debug-models.workflow-endpoints",
)
assert "taoryx.trajectory.reference_mission_composition" not in sys.modules
assert "taoryx.trajectory.contract_probe_mission_composition" not in sys.modules
providers = catalog.build_mission_composition_provider_registry()
assert tuple(item.metadata.id for item in providers.providers) == (
    "taoryx.reference.mission-composition",
    "taoryx.debug.mission-composition-contract-probe",
)
assert tuple(item.id for item in providers.list_models("taoryx.reference.mission-composition")) == (
    "reference_ballistic_3dof",
    "reference_constant_velocity_waypoint_3dof",
)
assert tuple(item.id for item in providers.list_models("taoryx.debug.mission-composition-contract-probe")) == (
    "contract_probe_vehicle",
)
assert providers.provider("taoryx.reference.mission-composition").metadata.version == "0.1.0a0"
assert providers.provider("taoryx.debug.mission-composition-contract-probe").metadata.version == "0.1.0a0"
assert "taoryx_reference_models" not in sys.modules
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


def test_simple_aero_source_tree_imports_without_reference_models() -> None:
    """The focused low-fidelity workflow must not recover from the aggregate."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-simple-aero/src",
            "packages/taoryx-reference-models/src",
        )
    )
    script = """
import sys
from pathlib import Path

# Keep the aggregate source visible so this proves selected discovery does not
# merely pass because the compatibility package is absent from the process.
sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part.startswith("taoryx-")
        and part not in {"taoryx-simple-aero", "taoryx-reference-models"}
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.simple-aero",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.simple-aero",)
assert tuple(item.id for item in catalog.records("trajectory_provider")) == ("reference.point_mass",)
assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
    "taoryx.simple-aero.mission-composition",
)
assert tuple(item.id for item in catalog.records("mission_workflow_endpoint_catalog")) == (
    "taoryx.simple-aero.workflow-endpoints",
)
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx.trajectory.registry_mission_composition" not in sys.modules

providers = catalog.build_mission_composition_provider_registry()
provider = providers.provider("taoryx.simple-aero.mission-composition")
assert provider.metadata.version == "0.1.0a0"
assert tuple(item.id for item in provider.list_models()) == ("simple_aero",)
assert "taoryx_reference_models.plugin" not in sys.modules
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


def test_hummingbird_source_tree_imports_without_reference_models() -> None:
    """The first isolated family must not recover its runtime from the aggregate."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-hummingbird/src",
        )
    )
    script = """
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-daveml",
            "taoryx-debug-models",
            "taoryx-a320",
            "taoryx-f16",
            "taoryx-nesc",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-passive-bodies",
            "taoryx-simple-aero",
            "taoryx-dual-launch",
            "taoryx-x15",
            "taoryx-hl20",
                "taoryx-source-table-fixed-wing",
                "taoryx-cadac",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx.composition_episode import open_vehicle_composition_episode

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.hummingbird",)
assert tuple(item.id for item in catalog.records("model")) == ("hummingbird",)
providers = catalog.build_mission_composition_provider_registry()
assert "taoryx.trajectory.registry_mission_composition" not in sys.modules
assert tuple(item.metadata.id for item in providers.providers) == ("taoryx.hummingbird.mission-composition",)
assert tuple(item.id for item in providers.list_models("taoryx.hummingbird.mission-composition")) == ("hummingbird",)
plan = build_model_authoring_plan(
    providers,
    catalog.build_controller_tuning_campaign_registry(),
    "taoryx.hummingbird.mission-composition",
    "hummingbird",
    family_adapters=catalog.build_family_adapter_registry(),
    fidelity="pseudo_6dof",
    realization_id="pseudo_6dof",
    mission_template_id="multirotor_pad_box_yaw_recovery_land_v1",
)
assert plan["status"] == "ready_to_author"
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        Path.cwd() / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml"
    ),
    plugins=catalog,
)
with TemporaryDirectory() as output_dir:
    execution = execute_vehicle_composition_batch(composition, output_dir, plugins=catalog)
    assert execution.passed
episode = open_vehicle_composition_episode(composition, plugins=catalog)
assert episode.__class__.__module__ == "taoryx.hummingbird_composition_episode"
episode.close()
assert "taoryx_reference_models" not in sys.modules
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


def test_nesc_source_tree_imports_without_reference_models() -> None:
    """NESC owns its replay/runtime boundary while retaining its DAVE-ML parser dependency."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-nesc/src",
        )
    )
    script = """
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-debug-models",
            "taoryx-a320",
            "taoryx-f16",
            "taoryx-hummingbird",
            "taoryx-passive-bodies",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-simple-aero",
            "taoryx-x15",
            "taoryx-hl20",
            "taoryx-source-table-fixed-wing",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx_nesc.resources import model_resource_root

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.nesc",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.nesc",)
assert tuple(item.id for item in catalog.records("family_adapter")) == ("reference_nesc_two_stage_rocket",)
assert "taoryx_reference_models" not in sys.modules
assert "taoryx_daveml.plugin" not in sys.modules
providers = catalog.build_mission_composition_provider_registry()
assert tuple(item.metadata.id for item in providers.providers) == ("taoryx.nesc.mission-composition",)
plan = build_model_authoring_plan(
    providers,
    catalog.build_controller_tuning_campaign_registry(),
    "taoryx.nesc.mission-composition",
    "reference_nesc_two_stage_rocket",
    family_adapters=catalog.build_family_adapter_registry(),
    fidelity="pseudo_6dof",
    realization_id="pseudo_6dof",
    mission_template_id="staged_rocket_launch_target_state_v1",
)
assert plan["status"] == "ready_to_author"
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        model_resource_root() / "examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml"
    ),
    plugins=catalog,
)
with TemporaryDirectory() as output_dir:
    execution = execute_vehicle_composition_batch(composition, output_dir, plugins=catalog)
    assert execution.passed
assert "taoryx_reference_models" not in sys.modules
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


def test_f16_source_tree_imports_without_reference_models() -> None:
    """F-16 owns its reductions, controller screens, and source assets."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-f16/src",
        )
    )
    script = """
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-debug-models",
            "taoryx-a320",
            "taoryx-hummingbird",
            "taoryx-nesc",
            "taoryx-passive-bodies",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-simple-aero",
            "taoryx-x15",
            "taoryx-hl20",
            "taoryx-source-table-fixed-wing",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
import taoryx.vehicle_execution_parity_witnesses as parity_witnesses
from taoryx.vehicle_execution_parity_witnesses import validate_vehicle_execution_parity_witnesses
from taoryx_f16.resources import model_resource_root

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.f16",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.f16",)
assert tuple(item.id for item in catalog.records("family_adapter")) == ("f16_s119",)
assert "taoryx_reference_models" not in sys.modules
assert "taoryx_daveml.plugin" not in sys.modules
assert "taoryx.source_f16" not in sys.modules
assert "taoryx.f16_reduced_execution" not in sys.modules
assert "taoryx.f16_composition_episode" not in sys.modules
assert "taoryx.f16_trim_evidence" not in sys.modules
providers = catalog.build_mission_composition_provider_registry()
assert tuple(item.metadata.id for item in providers.providers) == ("taoryx.f16.mission-composition",)
plan = build_model_authoring_plan(
    providers,
    catalog.build_controller_tuning_campaign_registry(),
    "taoryx.f16.mission-composition",
    "f16_s119",
    family_adapters=catalog.build_family_adapter_registry(),
    fidelity="pseudo_6dof",
    realization_id="pseudo_6dof",
    mission_template_id="powered_fixed_wing_racetrack_v1",
)
assert plan["status"] == "ready_to_author"
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        model_resource_root() / "examples/vehicle_composition/f16_racetrack_capability_pseudo6dof_compose.yaml"
    )
)
with TemporaryDirectory() as output_dir:
    execution = execute_vehicle_composition_batch(composition, output_dir, plugins=catalog)
    assert execution.passed
episode = open_vehicle_composition_episode(composition, plugins=catalog)
assert episode.__class__.__module__ == "taoryx.f16_composition_episode"
episode.close()
parity = validate_vehicle_execution_parity_witnesses(family_ids=("f16_s119",), plugins=catalog)
assert parity["status"] == "pass", parity
assert Path(parity_witnesses.__file__).resolve().is_relative_to(Path.cwd() / "src")
assert "taoryx_reference_models" not in sys.modules
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


def test_a320_source_tree_imports_without_reference_models() -> None:
    """A320 owns its OpenAP data, control campaigns, and reduced runtime."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-a320/src",
        )
    )
    script = """
import sys
from pathlib import Path
from tempfile import TemporaryDirectory

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-daveml",
            "taoryx-debug-models",
            "taoryx-f16",
            "taoryx-hummingbird",
            "taoryx-nesc",
            "taoryx-passive-bodies",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-simple-aero",
            "taoryx-x15",
            "taoryx-hl20",
            "taoryx-source-table-fixed-wing",
        }
        for part in Path(entry).parts
    )
]
import taoryx.plugins.discovery as plugin_discovery
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.model_authoring import build_model_authoring_plan
from taoryx.plugins import discover_plugins
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request
from taoryx_a320.resources import model_resource_root

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.a320",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.a320",)
assert tuple(item.id for item in catalog.records("family_adapter")) == ("a320_openap_3dof",)
assert tuple(item.id for item in catalog.records("local_native_coordinate_lqi_screen_definition")) == (
    "a320-pseudo-cruise-native-coordinate-lqi-screen-v1",
)
assert "taoryx_reference_models" not in sys.modules
assert "taoryx.trajectory.a320_openap" not in sys.modules
assert "taoryx.a320_reduced_execution" not in sys.modules
assert "taoryx.a320_composition_episode" not in sys.modules
native_lqi = catalog.build_local_native_coordinate_lqi_screen_registry().definitions
assert len(native_lqi) == 1
assert native_lqi[0].family_id == "a320_openap_3dof"
assert native_lqi[0].capability_adapter_id == "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1"
assert "taoryx.trajectory.a320_adapter" not in sys.modules
providers = catalog.build_mission_composition_provider_registry()
assert tuple(item.metadata.id for item in providers.providers) == ("taoryx.a320.mission-composition",)
plan = build_model_authoring_plan(
    providers,
    catalog.build_controller_tuning_campaign_registry(),
    "taoryx.a320.mission-composition",
    "a320_openap_3dof",
    family_adapters=catalog.build_family_adapter_registry(),
    fidelity="pseudo_6dof",
    realization_id="pseudo_6dof",
    mission_template_id="powered_fixed_wing_racetrack_v1",
)
assert plan["status"] == "ready_to_author"
native_plan = build_model_authoring_plan(
    providers,
    catalog.build_controller_tuning_campaign_registry(),
    "taoryx.a320.mission-composition",
    "a320_openap_3dof",
    family_adapters=catalog.build_family_adapter_registry(),
    local_controller_screens=catalog.build_local_controller_screen_advertisement_registry(),
    fidelity="pseudo_6dof",
    realization_id="jsbsim_surrogate_composite_pseudo6dof",
    mission_template_id="a320_local_native_coordinate_lqi_screen_v1",
)
assert native_plan["controller_automation"]["local_controller_screen"]["id"] == "a320-pseudo-cruise-native-coordinate-lqi-screen-v1"
composition = compile_vehicle_composition(
    load_vehicle_composition_request(
        model_resource_root() / "examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml"
    )
)
with TemporaryDirectory() as output_dir:
    execution = execute_vehicle_composition_batch(composition, output_dir, plugins=catalog)
    assert execution.passed
episode = open_vehicle_composition_episode(composition, plugins=catalog)
assert episode.__class__.__module__ == "taoryx.a320_composition_episode"
episode.close()
assert "taoryx_reference_models" not in sys.modules
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


def test_selected_plugin_scope_avoids_unselected_source_imports() -> None:
    """A focused host must not initialize sibling plug-ins merely to ignore them."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-debug-models/src",
            "packages/taoryx-a320/src",
            "packages/taoryx-f16/src",
            "packages/taoryx-hummingbird/src",
            "packages/taoryx-nesc/src",
            "packages/taoryx-passive-bodies/src",
            "packages/taoryx-source-table-fixed-wing/src",
            "packages/taoryx-x15/src",
            "packages/taoryx-hl20/src",
            "packages/taoryx-reference-models/src",
            "packages/taoryx-reachability/src",
            "packages/taoryx-simple-aero/src",
        )
    )
    script = """
import sys
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.hummingbird",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.hummingbird",)
assert "taoryx_hummingbird.plugin" in sys.modules
assert "taoryx_f16.plugin" not in sys.modules
assert "taoryx_nesc.plugin" not in sys.modules
assert "taoryx_passive_bodies.plugin" not in sys.modules
assert "taoryx_daveml.plugin" not in sys.modules
assert "taoryx_debug_models.plugin" not in sys.modules
assert "taoryx_a320.plugin" not in sys.modules
assert "taoryx_reachability.plugin" not in sys.modules
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx_simple_aero.plugin" not in sys.modules
assert "taoryx_source_table_fixed_wing.plugin" not in sys.modules
assert "taoryx_x15.plugin" not in sys.modules
assert "taoryx_hl20.plugin" not in sys.modules
assert "taoryx.composition_episode" not in sys.modules
assert "taoryx.hummingbird_composition_episode" not in sys.modules
assert "taoryx.hummingbird_composition_execution" not in sys.modules
assert "taoryx.hummingbird_reduced_interface" not in sys.modules
assert "taoryx.hummingbird_local_physical_control_screen" not in sys.modules
assert "taoryx.hummingbird_mission_capability" not in sys.modules
assert "taoryx.hummingbird_preflight" not in sys.modules
assert "taoryx.source_table_multirotor" not in sys.modules
assert "taoryx.family_adapter" not in sys.modules
assert "taoryx.control_allocation" not in sys.modules
assert "taoryx.vehicle_execution_preflight" not in sys.modules
assert "taoryx.composition_policy" not in sys.modules
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


def test_selected_x15_scope_avoids_reference_aggregate_imports() -> None:
    """The standalone X-15 plug-in must discover without loading reference code."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-x15/src",
            "packages/taoryx-reference-models/src",
            "packages/taoryx-reachability/src",
        )
    )
    script = """
import sys
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.x15",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.x15",)
assert "taoryx_x15.plugin" in sys.modules
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx_reachability.plugin" not in sys.modules
assert "taoryx.x15_adapter" not in sys.modules
assert "taoryx.x15_surface_authority_screen" not in sys.modules
assert "taoryx.x15_local_physical_surface_lqi_screen" not in sys.modules
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


def test_selected_hl20_scope_avoids_reference_and_reachability_imports() -> None:
    """The standalone HL-20 plug-in must discover without aggregate ownership."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-daveml/src",
            "packages/taoryx-hl20/src",
            "packages/taoryx-reference-models/src",
            "packages/taoryx-reachability/src",
        )
    )
    script = """
import sys
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.hl20",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.hl20",)
assert "taoryx_hl20.plugin" in sys.modules
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx_reachability.plugin" not in sys.modules
assert "taoryx.hl20_adapter" not in sys.modules
assert "taoryx.hl20_surface_authority_screen" not in sys.modules
assert "taoryx.hl20_local_physical_surface_lqi_screen" not in sys.modules
assert "taoryx.hl20_trim_evidence" not in sys.modules
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


def test_selected_source_table_fixed_wing_scope_avoids_reference_aggregate_imports() -> None:
    """X8/B747 discovery advertises both focused APIs without loading the aggregate or a plant."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-source-table-fixed-wing/src",
            "packages/taoryx-reference-models/src",
            "packages/taoryx-reachability/src",
        )
    )
    script = """
import sys
import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.source-table-fixed-wing",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.source-table-fixed-wing",)
assert tuple(item.id for item in catalog.records("family_adapter")) == ("skywalker_x8", "b747")
assert tuple(item.id for item in catalog.records("model")) == ("skywalker_x8", "b747")
assert "taoryx_reference_models.plugin" not in sys.modules
assert "taoryx_reachability.plugin" not in sys.modules
assert "taoryx.source_table_fixed_wing" not in sys.modules
assert "taoryx.source_table_fixed_wing_mission_capability" not in sys.modules

providers = catalog.build_mission_composition_provider_registry()
assert tuple(item.metadata.id for item in providers.providers) == (
    "taoryx.x8.mission-composition",
    "taoryx.b747.mission-composition",
)
assert tuple(item.id for item in providers.list_models("taoryx.x8.mission-composition")) == ("skywalker_x8",)
assert tuple(item.id for item in providers.list_models("taoryx.b747.mission-composition")) == ("b747",)
assert "taoryx_reference_models" not in sys.modules
assert "taoryx.source_table_fixed_wing" not in sys.modules
assert "taoryx.source_table_fixed_wing_mission_capability" not in sys.modules
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


def test_passive_bodies_source_tree_imports_without_parent_or_reachability_packages() -> None:
    """The released-body plug-in stays reusable without a parent package."""

    root = Path(__file__).resolve().parents[2]
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(root / path)
        for path in (
            "src",
            "packages/taoryx-passive-bodies/src",
        )
    )
    script = """
import sys
from pathlib import Path

import taoryx.plugins.discovery as plugin_discovery
from taoryx.plugins import discover_plugins

sys.path[:] = [
    entry
    for entry in sys.path
    if not any(
        part in {
            "taoryx-daveml",
            "taoryx-debug-models",
            "taoryx-a320",
            "taoryx-f16",
            "taoryx-hummingbird",
            "taoryx-nesc",
            "taoryx-reference-models",
            "taoryx-reachability",
            "taoryx-simple-aero",
            "taoryx-x15",
            "taoryx-hl20",
            "taoryx-source-table-fixed-wing",
        }
        for part in Path(entry).parts
    )
]
plugin_discovery._installed_entry_points = lambda: ()
catalog = discover_plugins(include_external=False, selected=("taoryx.passive-bodies",))
assert tuple(item.id for item in catalog.plugins) == ("taoryx.passive-bodies",)
assert tuple(item.id for item in catalog.records("family_adapter")) == ("tumbling_body",)
assert tuple(item.id for item in catalog.records("deployment_child_runtime")) == (
    "taoryx.passive-bodies.local-atmosphere-release.v1",
)
assert "taoryx_reference_models" not in sys.modules
assert "taoryx_reachability" not in sys.modules
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


def test_reference_models_package_has_no_generic_host_implementation() -> None:
    """Compatibility data must not supply core-wide validation code."""

    root = Path(__file__).resolve().parents[2]
    reference_host = root / "packages/taoryx-reference-models/src/taoryx"
    generic_modules = (
        "mission_composition_completion.py",
        "mission_composition_maturity.py",
        "vehicle_execution_parity_witnesses.py",
        "vehicle_integration_pilot.py",
        "vehicle_integration_pipeline.py",
    )

    assert not tuple(reference_host.glob("*.py"))
    assert all((root / "src/taoryx" / module).is_file() for module in generic_modules)
    ####


def test_packaged_plugin_assets_preserve_explicit_family_ownership() -> None:
    root = Path(__file__).resolve().parents[2]
    reference_data = root / "packages/taoryx-reference-models/src/taoryx_reference_models/data"
    f16_data = root / "packages/taoryx-f16/src/taoryx_f16/data"
    hummingbird_data = root / "packages/taoryx-hummingbird/src/taoryx_hummingbird/data"
    nesc_data = root / "packages/taoryx-nesc/src/taoryx_nesc/data"
    passive_data = root / "packages/taoryx-passive-bodies/src/taoryx_passive_bodies/data"
    debug_models_data = root / "packages/taoryx-debug-models/src/taoryx_debug_models/data"
    simple_aero_data = root / "packages/taoryx-simple-aero/src/taoryx_simple_aero/data"
    dual_launch_data = root / "packages/taoryx-dual-launch/src/taoryx_dual_launch/data"
    source_table_fixed_wing_data = root / "packages/taoryx-source-table-fixed-wing/src/taoryx_source_table_fixed_wing/data"
    x15_data = root / "packages/taoryx-x15/src/taoryx_x15/data"
    hl20_data = root / "packages/taoryx-hl20/src/taoryx_hl20/data"
    reachability_data = root / "packages/taoryx-reachability/src/taoryx_reachability/data"
    required_reference_registries = (
        "verification/horizontal_fidelity_registry.yaml",
        "verification/interface_channel_value_space_catalog.yaml",
        "verification/parameter_value_space_catalog.yaml",
        "verification/pseudo6dof_profiles.yaml",
        "verification/truth_objective_channel_value_space_catalog.yaml",
        "verification/vehicle_composition_registry.yaml",
        "verification/vehicle_execution_bindings.yaml",
        "verification/vehicle_endpoint_specs.yaml",
        "verification/vehicle_execution_parity.yaml",
        "verification/lqr_scaling_profiles.yaml",
        "verification/vehicle_models.yaml",
    )
    assert all((reference_data / relative).is_file() for relative in required_reference_registries)
    required_hummingbird_registries = (
        "verification/horizontal_fidelity_registry.yaml",
        "verification/lqr_scaling_profiles.yaml",
        "verification/pseudo6dof_profiles.yaml",
        "verification/vehicle_composition_registry.yaml",
        "verification/vehicle_endpoint_specs.yaml",
        "verification/vehicle_execution_bindings.yaml",
        "verification/vehicle_execution_parity.yaml",
        "verification/vehicle_execution_witnesses.yaml",
        "verification/vehicle_maturity_registry.yaml",
        "verification/vehicle_models.yaml",
    )
    assert all((hummingbird_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((f16_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((nesc_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((passive_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((source_table_fixed_wing_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((x15_data / relative).is_file() for relative in required_hummingbird_registries)
    assert all((hl20_data / relative).is_file() for relative in required_hummingbird_registries)
    required_source_table_route_assets = (
        "verification/family_qualification_missions.yaml",
        "verification/racetrack_templates.yaml",
    )
    assert all((source_table_fixed_wing_data / relative).is_file() for relative in required_source_table_route_assets)
    required_dual_launch_assets = (
        "verification/dual_launch_family_catalog.yaml",
        "verification/mission_workflow_endpoint_specs.yaml",
        "verification/vehicle_maturity_registry.yaml",
        "verification/workflow_endpoint_witnesses/dual_launch_attached_booster.yaml",
    )
    assert all((dual_launch_data / relative).is_file() for relative in required_dual_launch_assets)

    reference_models = yaml.safe_load((reference_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    f16_models = yaml.safe_load((f16_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    hummingbird_models = yaml.safe_load((hummingbird_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    nesc_models = yaml.safe_load((nesc_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    passive_models = yaml.safe_load((passive_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    source_table_fixed_wing_models = yaml.safe_load(
        (source_table_fixed_wing_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8")
    )
    x15_models = yaml.safe_load((x15_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    hl20_models = yaml.safe_load((hl20_data / "verification/vehicle_models.yaml").read_text(encoding="utf-8"))
    assert "hummingbird" not in reference_models["vehicles"]
    assert "f16_s119" not in reference_models["vehicles"]
    assert "reference_nesc_two_stage_rocket" not in reference_models["vehicles"]
    assert "tumbling_body" not in reference_models["vehicles"]
    assert "skywalker_x8" not in reference_models["vehicles"]
    assert "b747" not in reference_models["vehicles"]
    assert f16_models["vehicles"] == {}
    assert tuple(hummingbird_models["vehicles"]) == ("hummingbird",)
    assert nesc_models["vehicles"] == {}
    assert passive_models["vehicles"] == {}
    assert tuple(source_table_fixed_wing_models["vehicles"]) == ("b747", "skywalker_x8")
    assert tuple(x15_models["vehicles"]) == ("x15",)
    assert hl20_models["vehicles"] == {}
    source_table_missions = yaml.safe_load(
        (source_table_fixed_wing_data / "verification/family_qualification_missions.yaml").read_text(encoding="utf-8")
    )
    source_table_routes = yaml.safe_load(
        (source_table_fixed_wing_data / "verification/racetrack_templates.yaml").read_text(encoding="utf-8")
    )
    assert {mission["family"] for mission in source_table_missions["missions"]} == {"b747", "skywalker_x8"}
    assert {
        binding["vehicle_id"]
        for binding in source_table_routes["bindings"].values()
    } == {"b747", "skywalker_x8"}
    for mission in source_table_missions["missions"]:
        assert (source_table_fixed_wing_data / mission["problem"]).is_file()
        assert all((source_table_fixed_wing_data / table).is_file() for table in mission["tables"])
    ownership_text_suffixes = frozenset({".csv", ".json", ".md", ".prb", ".py", ".tbl", ".tex", ".txt", ".xml", ".yaml", ".yml"})
    assert not any("x15" in str(path.relative_to(reference_data)).casefold() for path in reference_data.rglob("*") if path.is_file())
    reference_text = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for path in reference_data.rglob("*")
        if path.is_file() and path.suffix.casefold() in ownership_text_suffixes
    ).casefold()
    assert "hummingbird" not in reference_text
    assert "f16_s119" not in reference_text
    assert "reference_f16_s119" not in reference_text
    assert "reference_nesc_two_stage_rocket" not in reference_text
    assert "skywalker_x8" not in reference_text
    assert "b747" not in reference_text
    assert "x15" not in reference_text
    assert "hl20" not in reference_text
    assert "dual_launch_glider" not in reference_text
    source_table_fixed_wing_text = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for path in source_table_fixed_wing_data.rglob("*")
        if path.is_file() and path.suffix.casefold() in ownership_text_suffixes
    ).casefold()
    for foreign_family in (
        "hummingbird",
        "f16_s119",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "x15",
        "hl20",
    ):
        assert foreign_family not in source_table_fixed_wing_text
    def has_transient_tool_cache(data_root: Path) -> bool:
        """Identify files that must never become a packaged model asset."""

        return any(
            "__pycache__" in packaged.parts
            or ".ruff_cache" in packaged.parts
            or packaged.suffix == ".pyc"
            for packaged in data_root.rglob("*")
        )
        ####

    assert not has_transient_tool_cache(f16_data)
    assert not has_transient_tool_cache(hummingbird_data)
    assert not has_transient_tool_cache(nesc_data)
    assert not has_transient_tool_cache(passive_data)
    assert not has_transient_tool_cache(source_table_fixed_wing_data)
    assert not has_transient_tool_cache(x15_data)
    assert not has_transient_tool_cache(hl20_data)
    assert not has_transient_tool_cache(dual_launch_data)

    hummingbird_provenance = json.loads((hummingbird_data / "package_data_provenance.json").read_text(encoding="utf-8"))
    assert hummingbird_provenance["schema"] == "taoryx.hummingbird-package-data/v1"
    assert hummingbird_provenance["family_id"] == "hummingbird"

    simple_aero_provenance = json.loads((simple_aero_data / "package_data_provenance.json").read_text(encoding="utf-8"))
    assert simple_aero_provenance["schema"] == "taoryx.simple-aero-package-data/v1"
    assert simple_aero_provenance["endpoint_id"] == "simple-aero-fixed-ld-batch"
    simple_aero_workflows = yaml.safe_load(
        (simple_aero_data / "verification/mission_workflow_endpoint_specs.yaml").read_text(encoding="utf-8")
    )
    assert [item["id"] for item in simple_aero_workflows["endpoints"]] == ["simple-aero-fixed-ld-batch"]
    assert (simple_aero_data / "verification/workflow_endpoint_witnesses/simple_aero_fixed_ld_baseline.yaml").is_file()
    assert not has_transient_tool_cache(simple_aero_data)

    dual_launch_provenance = json.loads((dual_launch_data / "package_data_provenance.json").read_text(encoding="utf-8"))
    assert dual_launch_provenance["schema"] == "taoryx.dual-launch-package-data/v1"
    assert dual_launch_provenance["family_id"] == "dual_launch_glider"
    assert dual_launch_provenance["endpoint_id"] == "dual-launch-attached-booster-batch"
    dual_launch_families = yaml.safe_load(
        (dual_launch_data / "verification/dual_launch_family_catalog.yaml").read_text(encoding="utf-8")
    )
    dual_launch_maturity = yaml.safe_load(
        (dual_launch_data / "verification/vehicle_maturity_registry.yaml").read_text(encoding="utf-8")
    )
    dual_launch_workflows = yaml.safe_load(
        (dual_launch_data / "verification/mission_workflow_endpoint_specs.yaml").read_text(encoding="utf-8")
    )
    assert [item["family_id"] for item in dual_launch_families["families"]] == ["dual_launch_glider"]
    assert [item["id"] for item in dual_launch_maturity["records"]] == ["dual_launch_glider"]
    assert [item["id"] for item in dual_launch_workflows["endpoints"]] == ["dual-launch-attached-booster-batch"]
    assert not (reference_data / "verification/mission_workflow_endpoint_specs.yaml").exists()

    debug_models_provenance = json.loads((debug_models_data / "package_data_provenance.json").read_text(encoding="utf-8"))
    assert debug_models_provenance["schema"] == "taoryx.debug-models-package-data/v1"
    assert debug_models_provenance["endpoint_ids"] == [
        "reference-ballistic-3dof-batch",
        "reference-waypoint-3dof-batch",
        "debug-contract-probe-batch",
    ]
    debug_model_workflows = yaml.safe_load(
        (debug_models_data / "verification/mission_workflow_endpoint_specs.yaml").read_text(encoding="utf-8")
    )
    assert [item["id"] for item in debug_model_workflows["endpoints"]] == debug_models_provenance["endpoint_ids"]
    for witness in (
        "reference_ballistic_3dof.yaml",
        "reference_waypoint_3dof.yaml",
        "debug_contract_probe.yaml",
    ):
        assert (debug_models_data / "verification/workflow_endpoint_witnesses" / witness).is_file()
    assert not has_transient_tool_cache(debug_models_data)

    reachability_provenance = json.loads((reachability_data / "package_data_provenance.json").read_text(encoding="utf-8"))
    assert reachability_provenance["schema"] == "taoryx.reachability-package-data/v1"
    assert {
        "x15_overlay/verification/vehicle_composition_registry.yaml",
        "hl20_overlay/verification/vehicle_composition_registry.yaml",
        "x15_overlay/examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
        "hl20_overlay/examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
        "hl20_overlay/examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
    } <= set(reachability_provenance["asset_files"])
    assert (reachability_data / "verification/reachability_profile_catalog.yaml").is_file()
    assert (reachability_data / "x15_overlay/verification/vehicle_execution_bindings.yaml").is_file()
    assert (reachability_data / "hl20_overlay/verification/vehicle_execution_bindings.yaml").is_file()
    assert not has_transient_tool_cache(reachability_data)

    for data_root, excluded_prefixes in (
        (
            reference_data,
            (
                "verification/",
            ),
        ),
        (
            hummingbird_data,
            (
                "package_data_provenance.json",
                "verification/",
                "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/catalog.yaml",
                "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/manifest.json",
            ),
        ),
        (
            f16_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
        (
            nesc_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
        (
            simple_aero_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
        (
            debug_models_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
        (
            source_table_fixed_wing_data,
            (
                "package_data_provenance.json",
                "verification/",
                "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/catalog.yaml",
                "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/manifest.json",
            ),
        ),
        (
            x15_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
        (
            hl20_data,
            (
                "package_data_provenance.json",
                "verification/",
            ),
        ),
    ):
        for packaged in data_root.rglob("*"):
            if not packaged.is_file() or packaged.suffix == ".pyc":
                continue
            relative = packaged.relative_to(data_root).as_posix()
            if relative.startswith(excluded_prefixes):
                continue
            canonical = root / relative
            assert canonical.is_file(), f"packaged asset has no canonical source: {relative}"
            assert cmp(packaged, canonical, shallow=False), f"packaged asset drifted from canonical source: {relative}"
    ####
