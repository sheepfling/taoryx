"""Focused optional-reachability plug-in boundary tests."""

from __future__ import annotations

import os
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest
from taoryx_x15.resources import model_resource_root

from taoryx.plugins import VehicleCatalogOverlayFragment, discover_plugins
from taoryx.vehicle_catalog_resources import vehicle_catalog_resource, vehicle_catalog_resources
from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog
from tools.extract_reachability_plugin_assets import check as check_reachability_plugin_assets
from tools.extract_reachability_plugin_assets import extract as extract_reachability_plugin_assets

ROOT = Path(__file__).resolve().parents[3]
X15_PLUGIN_ID = "taoryx.x15"
REACHABILITY_PLUGIN_ID = "taoryx.reachability"
X15_FAMILY_ID = "x15"
X15_STAGED_MISSION_ID = "x15_staged_booster_reachability_v1"
HL20_PLUGIN_ID = "taoryx.hl20"
HL20_FAMILY_ID = "hl20_mod_k"
HL20_REACHABILITY_MISSION_IDS = {
    "hl20_source_booster_release_replay_v1",
    "lifting_body_glide_energy_management_v1",
}


def test_reachability_packaged_data_is_a_current_exact_extract(tmp_path: Path) -> None:
    """The optional overlay wheel contains only its profile and showcase assets."""

    target = tmp_path / "data"
    extract_reachability_plugin_assets(target)
    assert check_reachability_plugin_assets(target) == ()

    stale = target / "obsolete-reachability-asset.txt"
    stale.write_text("stale", encoding="utf-8")
    assert check_reachability_plugin_assets(target) == ("unexpected generated reachability asset: obsolete-reachability-asset.txt",)
    ####


def test_reachability_has_no_compatibility_aggregate_distribution_dependency() -> None:
    """Overlay ownership is directly on X-15 and HL-20, never the aggregate."""

    project = tomllib.loads((ROOT / "packages/taoryx-reachability/pyproject.toml").read_text(encoding="utf-8"))
    dependencies = tuple(project["project"]["dependencies"])

    assert all(not dependency.startswith("taoryx-reference-models") for dependency in dependencies)
    assert any(dependency.startswith("taoryx-x15") for dependency in dependencies)
    assert any(dependency.startswith("taoryx-hl20") for dependency in dependencies)
    ####


def test_reachability_catalog_overlays_require_and_extend_their_base_fragments() -> None:
    """Each overlay appears only with its own selected vehicle base."""

    overlay_only = discover_plugins(include_external=False, selected=(REACHABILITY_PLUGIN_ID,))
    for contribution_id, base_fragment_id, family_id, resource_root in (
        ("taoryx.reachability.x15-staged-catalog-overlay", "taoryx.x15.vehicle-catalog", X15_FAMILY_ID, "data/x15_overlay"),
        ("taoryx.reachability.hl20-catalog-overlay", "taoryx.hl20.vehicle-catalog", HL20_FAMILY_ID, "data/hl20_overlay"),
    ):
        overlay = overlay_only.contribution("vehicle_catalog_overlay_fragment", contribution_id).value
        assert isinstance(overlay, VehicleCatalogOverlayFragment)
        assert overlay.resource_package == "taoryx_reachability"
        assert overlay.extends_fragment_ids == (base_fragment_id,)
        assert overlay.resource_root == resource_root
        assert overlay.family_ids == (family_id,)
    with pytest.raises(FileNotFoundError, match="requirements are inactive"):
        vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=overlay_only)

    plugins = discover_plugins(include_external=False, selected=(X15_PLUGIN_ID, REACHABILITY_PLUGIN_ID))
    resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=plugins)
    assert resources[0] == model_resource_root() / "verification/vehicle_composition_registry.yaml"
    assert len(resources) == 2
    assert "taoryx_reachability" in resources[1].parts
    assert X15_STAGED_MISSION_ID in {
        item.id for item in load_resolved_vehicle_composition_catalog(plugins=plugins).vehicle(X15_FAMILY_ID).declaration.mission_templates
    }
    assert {
        (item.mission, item.fidelity)
        for item in load_vehicle_execution_binding_catalog(plugins=plugins).bindings
        if item.mission == X15_STAGED_MISSION_ID
    } == {
        (X15_STAGED_MISSION_ID, "point_mass_3dof"),
        (X15_STAGED_MISSION_ID, "pseudo_6dof"),
    }
    assert {item.id for item in load_vehicle_execution_witness_catalog(plugins=plugins).witnesses} >= {
        "x15-staged-3dof-batch",
        "x15-staged-pseudo6dof-batch",
    }
    staged_request = vehicle_catalog_resource(
        "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
        plugins=plugins,
    )
    assert "x15_overlay" in staged_request.parts

    hl20_plugins = discover_plugins(include_external=False, selected=(HL20_PLUGIN_ID, REACHABILITY_PLUGIN_ID))
    hl20_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=hl20_plugins)
    assert len(hl20_resources) == 2
    assert "taoryx_hl20" in hl20_resources[0].parts
    assert "hl20_overlay" in hl20_resources[1].parts
    hl20_missions = {
        item.id
        for item in load_resolved_vehicle_composition_catalog(plugins=hl20_plugins)
        .vehicle(HL20_FAMILY_ID)
        .declaration.mission_templates
    }
    assert HL20_REACHABILITY_MISSION_IDS <= hl20_missions
    assert {
        (item.mission, item.fidelity)
        for item in load_vehicle_execution_binding_catalog(plugins=hl20_plugins).bindings
        if item.mission in HL20_REACHABILITY_MISSION_IDS
    } == {
        ("hl20_source_booster_release_replay_v1", "point_mass_3dof"),
        ("hl20_source_booster_release_replay_v1", "pseudo_6dof"),
        ("lifting_body_glide_energy_management_v1", "rigid_body_6dof_direct_wrench"),
        ("lifting_body_glide_energy_management_v1", "rigid_body_6dof_surface_allocated"),
    }
    assert {"hl20-source-release-3dof-batch", "hl20-source-release-pseudo6dof-batch"} <= {
        item.id for item in load_vehicle_execution_witness_catalog(plugins=hl20_plugins).witnesses
    }
    source_release_request = vehicle_catalog_resource(
        "examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
        plugins=hl20_plugins,
    )
    glide_energy_request = vehicle_catalog_resource(
        "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
        plugins=hl20_plugins,
    )
    assert "hl20_overlay" in source_release_request.parts
    assert "hl20_overlay" in glide_energy_request.parts

    combined_plugins = discover_plugins(
        include_external=False,
        selected=(X15_PLUGIN_ID, HL20_PLUGIN_ID, REACHABILITY_PLUGIN_ID),
    )
    combined_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=combined_plugins)
    assert len(combined_resources) == 4
    assert sum("x15_overlay" in source.parts for source in combined_resources) == 1
    assert sum("hl20_overlay" in source.parts for source in combined_resources) == 1
    combined_catalog = load_resolved_vehicle_composition_catalog(plugins=combined_plugins)
    assert X15_STAGED_MISSION_ID in {
        item.id for item in combined_catalog.vehicle(X15_FAMILY_ID).declaration.mission_templates
    }
    assert HL20_REACHABILITY_MISSION_IDS <= {
        item.id for item in combined_catalog.vehicle(HL20_FAMILY_ID).declaration.mission_templates
    }
    ####


def test_reachability_discovery_defers_overlay_implementations_until_a_mission_selects_one() -> None:
    """A selected reachability plug-in advertises stable identities without loading plants."""

    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(ROOT / "src"),
            str(ROOT / "packages/taoryx-daveml/src"),
            str(ROOT / "packages/taoryx-x15/src"),
            str(ROOT / "packages/taoryx-hl20/src"),
            str(ROOT / "packages/taoryx-reachability/src"),
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
]

from taoryx.plugins import DeferredSemanticPreflightHandler, discover_plugins
from taoryx_reachability.plugin import PLUGIN


@dataclass(frozen=True)
class EntryPoint:
    name: str
    value: str
    target: object

    def load(self) -> object:
        return self.target


assert "taoryx_reachability.capabilities" not in sys.modules
assert "taoryx_reachability.preflight" not in sys.modules
assert "taoryx.hl20_source_aerodynamics" not in sys.modules
catalog = discover_plugins(
    include_builtin=False,
    entry_points=(EntryPoint("taoryx.reachability", "taoryx_reachability.plugin:PLUGIN", PLUGIN),),
)
assert tuple(item.id for item in catalog.plugins) == ("taoryx.reachability",)
assert catalog.plugin_revision("taoryx.reachability").version == "0.1.0a0"
assert "taoryx_reachability.capabilities" not in sys.modules
assert "taoryx_reachability.preflight" not in sys.modules
assert "taoryx.hl20_source_aerodynamics" not in sys.modules
assert "taoryx_reference_models" not in sys.modules
assert tuple(item.id for item in catalog.records("mission_capability_adapter")) == (
    "taoryx.x15_staged_reachability.capability.v1",
    "taoryx.hl20_glide_energy.capability.v1",
    "taoryx.hl20_source_booster_release_replay.capability.v1",
)
assert all(isinstance(item.value, DeferredSemanticPreflightHandler) for item in catalog.records("semantic_preflight_handler"))
workbench = catalog.build_reachability_provider_registry().provider("taoryx.reachability.workbench")
assert workbench.metadata.families == ("hl20_mod_k", "x15")
adapter = catalog.contribution("mission_capability_adapter", "taoryx.x15_staged_reachability.capability.v1").value
assert adapter.supports(
    SimpleNamespace(
        family_id="x15",
        mission="x15_staged_booster_reachability_v1",
        fidelity="point_mass_3dof",
    )
)
assert "taoryx_reachability.capabilities" in sys.modules
assert "taoryx.hl20_source_aerodynamics" in sys.modules
assert "taoryx_reference_models" not in sys.modules
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
