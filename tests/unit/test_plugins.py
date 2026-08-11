from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import dataclass
from filecmp import cmp
from pathlib import Path

import pytest
from taoryx_simple_aero.plugin import PLUGIN as SIMPLE_AERO_PLUGIN

import taoryx.runtime.cli as runtime_cli
from taoryx.composition_episode import registered_episode_factory_ids
from taoryx.plugins import (
    PluginCollisionError,
    PluginCompatibilityError,
    PluginDefinition,
    PluginMetadata,
    PluginRegistrar,
    discover_plugins,
)
from taoryx.runtime.cli import main
from taoryx.vehicle_batch_execution import registered_vehicle_batch_factory_ids
from taoryx.vehicle_runtime_lowering import build_vehicle_runtime_adapter_registry


@dataclass(frozen=True)
class _EntryPoint:
    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target
        ####

    ####


def _model_plugin(plugin_id: str, model_id: str, *, api_version: str = "1") -> PluginDefinition:
    def register(registrar: PluginRegistrar) -> None:
        registrar.register_model(model_id, object())
        ####

    return PluginDefinition(
        PluginMetadata(
            id=plugin_id,
            package=plugin_id,
            version="1.0.0",
            api_version=api_version,
            description=f"Test plug-in {plugin_id}.",
        ),
        register,
    )
    ####


def test_bundled_vehicle_registrations_cross_the_plugin_boundary_without_id_drift() -> None:
    catalog = discover_plugins(include_external=False)

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.daveml",
        "taoryx.reachability",
        "taoryx.reference-models",
        "taoryx.simple-aero",
    )
    assert tuple(item.id for item in catalog.records("family_adapter")) == (
        "x15",
        "hl20_mod_k",
        "reference_nesc_two_stage_rocket",
        "tumbling_body",
        "skywalker_x8",
        "b747",
        "a320_openap_3dof",
        "f16_s119",
        "hummingbird",
    )
    assert {item.plugin.id for item in catalog.records("family_adapter")} == {"taoryx.reference-models"}
    assert tuple(item.id for item in catalog.records("model")) == tuple(item.id for item in catalog.records("family_adapter"))
    assert {item.plugin.id for item in catalog.records("model")} == {"taoryx.reference-models"}
    assert len(catalog.records("mission_capability_adapter")) == 32
    assert {item.plugin.id for item in catalog.records("mission_capability_adapter")} == {"taoryx.reachability", "taoryx.reference-models"}
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("mission_capability_adapter")) == 4
    assert sum(item.plugin.id == "taoryx.reference-models" for item in catalog.records("mission_capability_adapter")) == 28
    assert registered_vehicle_batch_factory_ids(plugins=catalog) == tuple(sorted(item.id for item in catalog.records("execution_factory")))
    assert registered_episode_factory_ids(plugins=catalog) == tuple(sorted(item.id for item in catalog.records("episode_factory")))
    assert len(catalog.records("execution_factory")) == 27
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("execution_factory")) == 3
    assert sum(item.plugin.id == "taoryx.reference-models" for item in catalog.records("execution_factory")) == 24
    assert sum(item.plugin.id == "taoryx.reachability" for item in catalog.records("semantic_preflight_handler")) == 4
    assert sum(item.plugin.id == "taoryx.reference-models" for item in catalog.records("semantic_preflight_handler")) == 28
    assert len(catalog.records("episode_factory")) == 6
    assert len(catalog.records("batch_episode_parity_verifier")) == 6
    assert tuple(item.id for item in catalog.records("controller_tuning_campaign")) == (
        "b747-source-surface-local-lqi-v1",
        "b747-language-backed-guidance-local-lqi-v1",
        "b747-language-backed-pseudo-guidance-local-lqi-v1",
        "x8-source-surface-local-lqi-v1",
        "x8-language-backed-guidance-local-lqi-v1",
        "x8-language-backed-pseudo-guidance-local-lqi-v1",
        "a320-point-cruise-performance-lqr-v1",
        "a320-pseudo-cruise-attitude-v1",
        "f16-point-source-trim-translation-v1",
        "f16-pseudo-source-trim-attitude-v1",
        "f16-source-surface-local-lqr-v1",
        "f16-source-surface-schedule-lqr-v1",
        "f16-source-surface-local-lqi-v1",
        "f16-source-surface-schedule-lqi-v1",
        "hummingbird-pseudo-hover-attitude-v1",
        "hummingbird-source-rotor-local-lqi-v1",
        "hummingbird-source-rotor-vertical-lqi-v1",
        "x15-source-release-direct-wrench-v1",
        "x15-source-release-direct-wrench-lqi-v1",
        "x15-source-surface-local-lqi-v1",
        "hl20-source-subsonic-direct-wrench-v1",
        "hl20-source-subsonic-direct-wrench-lqi-v1",
        "hl20-source-surface-local-lqi-v1",
    )
    assert {item.plugin.id for item in catalog.records("controller_tuning_campaign")} == {"taoryx.reference-models"}
    assert tuple(item.id for item in catalog.records("local_controller_screen_advertisement")) == (
        "f16-source-trim-direct-wrench-lqr-screen-v1",
    )
    assert {item.plugin.id for item in catalog.records("local_controller_screen_advertisement")} == {"taoryx.reference-models"}
    screens = catalog.build_local_controller_screen_advertisement_registry()
    assert [item.id for item in screens.matching(
        provider_id="taoryx.registry.mission-composition",
        model_id="f16_s119",
        family_id="f16_s119",
        fidelity="rigid_body_6dof_direct_wrench",
        realization_id="rigid_body_6dof_direct_wrench",
        mission_template_id="f16_local_physical_control_screen_v1",
    )] == ["f16-source-trim-direct-wrench-lqr-screen-v1"]
    assert tuple(item.id for item in catalog.records("mission_composition_provider")) == (
        "taoryx.registry.mission-composition",
        "taoryx.reference.mission-composition",
        "taoryx.debug.mission-composition-contract-probe",
    )
    composer_providers = catalog.build_mission_composition_provider_registry()
    assert tuple(item.metadata.id for item in composer_providers.providers) == (
        "taoryx.registry.mission-composition",
        "taoryx.reference.mission-composition",
        "taoryx.debug.mission-composition-contract-probe",
    )
    assert len(composer_providers.list_models("taoryx.registry.mission-composition")) == 11
    assert composer_providers.model("taoryx.registry.mission-composition", "f16_s119").id == "f16_s119"
    assert composer_providers.public_dict()["schema"] == "taoryx.mission-composition-provider-catalog/v1"
    assert tuple(item.id for item in catalog.records("reachability_provider")) == ("taoryx.reachability.workbench",)
    reachability = catalog.build_reachability_provider_registry().provider("taoryx.reachability.workbench")
    assert reachability.metadata.families == ("hl20_mod_k", "tumbling_body", "x15")

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
    providers = first.build_trajectory_provider_registry()
    assert providers.provider("reference.point_mass") is contribution.value
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
        "taoryx.daveml",
        "taoryx.reachability",
        "taoryx.reference-models",
        "taoryx.simple-aero",
    ]
    assert {item["kind"] for item in payload["contributions"]} == {
        "batch_episode_parity_verifier",
        "controller_tuning_campaign",
        "episode_factory",
        "execution_factory",
        "family_adapter",
        "local_controller_screen_advertisement",
        "mission_capability_adapter",
        "mission_composition_provider",
        "model",
        "model_format",
        "reachability_provider",
        "semantic_preflight_handler",
        "trajectory_provider",
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

    monkeypatch.setattr(runtime_cli, "discover_plugins", discover_installed)
    monkeypatch.setattr(runtime_cli.importlib_metadata, "version", lambda _name: "0.1.0a0")

    assert runtime_cli.main(["plugins", "check", "--profile", "full", "--json"]) == 0

    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.plugin-installation-check/v1"
    assert payload["ready"] is True
    assert [item["distribution"] for item in payload["distributions"]] == [
        "taoryx",
        "taoryx-daveml",
        "taoryx-simple-aero",
        "taoryx-reference-models",
        "taoryx-reachability",
    ]
    assert all(item["loaded"] for item in payload["plugins"])
    ####


def test_plugin_cli_fails_closed_for_missing_model_profile_distribution(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    empty_catalog = discover_plugins(include_builtin=False, include_external=False)

    monkeypatch.setattr(runtime_cli, "discover_plugins", lambda **_kwargs: empty_catalog)

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
        "taoryx-simple-aero",
        "taoryx-reference-models",
    ]
    assert all(not item["loaded"] for item in payload["plugins"])
    ####


def test_disabling_reference_models_removes_only_reference_owned_contributions() -> None:
    catalog = discover_plugins(
        include_external=False,
        disabled=("taoryx.reference-models",),
    )

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.daveml",
        "taoryx.reachability",
        "taoryx.simple-aero",
    )
    assert catalog.records("family_adapter") == ()
    assert catalog.records("model") == ()
    assert len(catalog.records("mission_capability_adapter")) == 4
    assert catalog.records("mission_composition_provider") == ()
    assert len(catalog.records("semantic_preflight_handler")) == 4
    assert len(catalog.records("execution_factory")) == 3
    assert catalog.records("episode_factory") == ()
    assert catalog.records("batch_episode_parity_verifier") == ()
    assert tuple(item.id for item in catalog.records("model_format")) == ("daveml",)
    assert tuple(item.id for item in catalog.records("trajectory_provider")) == ("reference.point_mass",)
    assert tuple(item.id for item in catalog.records("reachability_provider")) == ("taoryx.reachability.workbench",)
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [
        ("taoryx.daveml", "loaded"),
        ("taoryx.reachability", "loaded"),
        ("taoryx.reference-models", "disabled"),
        ("taoryx.simple-aero", "loaded"),
    ]
    ####


def test_disabling_reachability_removes_only_reachability_owned_overlays() -> None:
    catalog = discover_plugins(
        include_external=False,
        disabled=("taoryx.reachability",),
    )

    assert tuple(item.id for item in catalog.plugins) == (
        "taoryx.daveml",
        "taoryx.reference-models",
        "taoryx.simple-aero",
    )
    assert len(catalog.records("family_adapter")) == 9
    assert len(catalog.records("model")) == 9
    assert len(catalog.records("mission_capability_adapter")) == 28
    assert len(catalog.records("semantic_preflight_handler")) == 28
    assert len(catalog.records("execution_factory")) == 24
    assert len(catalog.records("episode_factory")) == 6
    assert len(catalog.records("batch_episode_parity_verifier")) == 6
    assert len(catalog.records("controller_tuning_campaign")) == 23
    assert len(catalog.records("local_controller_screen_advertisement")) == 1
    assert catalog.records("reachability_provider") == ()
    assert [(item.plugin_id, item.status) for item in catalog.diagnostics] == [
        ("taoryx.daveml", "loaded"),
        ("taoryx.reachability", "disabled"),
        ("taoryx.reference-models", "loaded"),
        ("taoryx.simple-aero", "loaded"),
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
            "packages/taoryx-simple-aero/src",
            "packages/taoryx-reference-models/src",
        )
    )
    script = """
import sys
from pathlib import Path

# Preserve the three requested editable source roots while excluding the
# reachability .pth entry installed by a full contributor bootstrap.
sys.path[:] = [
    entry
    for entry in sys.path
    if "taoryx-reachability" not in Path(entry).parts
]
from taoryx.plugins import discover_plugins

catalog = discover_plugins(include_external=False)
assert tuple(item.id for item in catalog.plugins) == (
    "taoryx.daveml",
    "taoryx.reference-models",
    "taoryx.simple-aero",
)
assert len(catalog.records("family_adapter")) == 9
assert len(catalog.records("mission_capability_adapter")) == 28
assert len(catalog.records("semantic_preflight_handler")) == 28
assert len(catalog.records("execution_factory")) == 24
assert len(catalog.records("local_controller_screen_advertisement")) == 1
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


def test_packaged_plugin_assets_match_their_canonical_sources() -> None:
    root = Path(__file__).resolve().parents[2]
    reference_data = root / "packages/taoryx-reference-models/src/taoryx_reference_models/data"
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
        "verification/vehicle_models.yaml",
    )
    assert all((reference_data / relative).is_file() for relative in required_reference_registries)
    data_roots = (
        root / "packages/taoryx-simple-aero/src/taoryx_simple_aero/data",
        reference_data,
        root / "packages/taoryx-reachability/src/taoryx_reachability/data",
    )

    for data_root in data_roots:
        packaged_files = tuple(path for path in data_root.rglob("*") if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc")
        assert packaged_files
        for packaged in packaged_files:
            relative = packaged.relative_to(data_root)
            canonical = root / relative
            assert canonical.is_file(), f"packaged asset has no canonical source: {relative}"
            assert cmp(packaged, canonical, shallow=False), f"packaged asset drifted from canonical source: {relative}"
    ####
