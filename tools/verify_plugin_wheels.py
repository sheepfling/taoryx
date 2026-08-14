"""Build and smoke-test isolated Taoryx plug-in wheels without source fallback."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
import zipfile
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True, slots=True)
class PluginWheelSpec:
    """One independently installable plug-in wheel to exercise."""

    selector: str
    project: Path
    wheel_stem: str
    plugin_id: str
    plugin_version: str
    owned_modules: tuple[str, ...]
    deferred_module_names: tuple[str, ...] = ()
    model_format_imports: tuple[tuple[str, str], ...] = ()
    adapter_builds: tuple[tuple[str, str], ...] = ()
    resource_module: str | None = None
    composition_witness: str | None = None
    additional_composition_witnesses: tuple[str, ...] = ()
    batch_only_witnesses: tuple[str, ...] = ()
    focused_provider_id: str | None = None
    focused_provider_models: tuple[tuple[str, tuple[str, ...]], ...] = ()
    excluded_mission_template_ids: tuple[str, ...] = ()
    trim_evidence_family_id: str | None = None
    exercise_episode: bool = True
    exercise_composition_batch: bool = True
    exercise_language_backed_materialization: bool = False
    wheel_dependency_selectors: tuple[str, ...] = ()
    include_by_default: bool = True
    deployment_child_runtime_id: str | None = None
    workflow_endpoint_ids: tuple[str, ...] = ()
    exercise_provider_catalog: bool = True
    forbidden_wheel_prefixes: tuple[str, ...] = ()
    forbidden_top_level_packages: tuple[str, ...] = ()
    required_wheel_paths: tuple[str, ...] = ()

    ####


PLUGIN_WHEEL_SPECS: tuple[PluginWheelSpec, ...] = (
    PluginWheelSpec(
        selector="daveml",
        project=ROOT / "packages" / "taoryx-daveml",
        wheel_stem="taoryx_daveml",
        plugin_id="taoryx.daveml",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.trajectory.daveml_evaluator",
            "taoryx.trajectory.daveml_import",
            "taoryx.trajectory.daveml_semantic",
            "taoryx_daveml.plugin",
        ),
        model_format_imports=(("daveml", "taoryx.trajectory.daveml_import"),),
        exercise_episode=False,
        exercise_provider_catalog=False,
        include_by_default=False,
    ),
    PluginWheelSpec(
        selector="cadac",
        project=ROOT / "packages" / "taoryx-cadac",
        wheel_stem="taoryx_cadac",
        plugin_id="taoryx.cadac",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.families.cadac.mission_composition_plugin",
            "taoryx.families.cadac.sensor_integration",
            "taoryx.families.cadac.table_conversion",
            "taoryx_cadac.plugin",
        ),
        focused_provider_models=(
            (
                "cadac",
                (
                    "cadac.ads6.aircraft",
                    "cadac.ads6.sam",
                    "cadac.ads6.srbm",
                    "cadac.agm6.aircraft",
                    "cadac.agm6.ground_target",
                    "cadac.agm6.missile",
                    "cadac.aim5.missile",
                    "cadac.aim5.target",
                    "cadac.cruise5.cruise_vehicle",
                    "cadac.falcon6.aircraft",
                    "cadac.ghame3.hypersonic_vehicle",
                    "cadac.ghame6.hypersonic_vehicle",
                    "cadac.ghame6.satellite",
                    "cadac.magsix.vehicle",
                    "cadac.rocket6g.launch_vehicle",
                    "cadac.sraam6.missile",
                    "cadac.sraam6.target",
                ),
            ),
        ),
        exercise_episode=False,
    ),
    PluginWheelSpec(
        selector="a320",
        project=ROOT / "packages" / "taoryx-a320",
        wheel_stem="taoryx_a320",
        plugin_id="taoryx.a320",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.a320_composition_episode",
            "taoryx.a320_local_native_coordinate_lqi",
            "taoryx.a320_mission_capability",
            "taoryx.a320_reduced_execution",
            "taoryx.trajectory.a320_adapter",
            "taoryx.trajectory.a320_openap",
            "taoryx.trajectory.a320_pseudo6dof",
            "taoryx.trajectory.a320_racetrack",
        ),
        deferred_module_names=(
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
        ),
        adapter_builds=(
            ("a320_openap_3dof", "point_mass_3dof"),
            ("a320_openap_3dof", "pseudo_6dof"),
        ),
        resource_module="taoryx_a320.resources",
        composition_witness="examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml",
        batch_only_witnesses=("examples/vehicle_composition/a320_local_native_coordinate_lqi_screen_compose.yaml",),
        forbidden_wheel_prefixes=(
            "taoryx_a320/data/resources/aerospace/daveml/taoryx-corpus-v1.1/qualified-models/",
            "taoryx_a320/data/resources/aerospace/daveml/taoryx-corpus-v1.1/source-corpora/",
        ),
    ),
    PluginWheelSpec(
        selector="f16",
        project=ROOT / "packages" / "taoryx-f16",
        wheel_stem="taoryx_f16",
        plugin_id="taoryx.f16",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.f16_composition_episode",
            "taoryx.f16_trim_evidence",
            "taoryx.f16_mission_capability",
            "taoryx.f16_mission_translation",
            "taoryx.f16_reduced_execution",
            "taoryx.source_f16",
            "taoryx.trajectory.f16_reduced_adapter",
            "taoryx.trajectory.f16_reference",
        ),
        deferred_module_names=(
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
        ),
        adapter_builds=(("f16_s119", "rigid_body_6dof_surface_allocated"),),
        resource_module="taoryx_f16.resources",
        composition_witness="examples/vehicle_composition/f16_racetrack_capability_pseudo6dof_compose.yaml",
        trim_evidence_family_id="reference_f16_s119",
        wheel_dependency_selectors=("daveml",),
    ),
    PluginWheelSpec(
        selector="hummingbird",
        project=ROOT / "packages" / "taoryx-hummingbird",
        wheel_stem="taoryx_hummingbird",
        plugin_id="taoryx.hummingbird",
        plugin_version="0.1.0a0",
        owned_modules=("taoryx.hummingbird_composition_episode",),
        deferred_module_names=(
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
        ),
        adapter_builds=(("hummingbird", "rigid_body_6dof_surface_allocated"),),
        resource_module="taoryx_hummingbird.resources",
        composition_witness="examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml",
        batch_only_witnesses=("examples/vehicle_composition/hummingbird_local_direct_wrench_screen_compose.yaml",),
    ),
    PluginWheelSpec(
        selector="source-table-fixed-wing",
        project=ROOT / "packages" / "taoryx-source-table-fixed-wing",
        wheel_stem="taoryx_source_table_fixed_wing",
        plugin_id="taoryx.source-table-fixed-wing",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.source_table_fixed_wing",
            "taoryx.source_table_fixed_wing_mission_capability",
            "taoryx.x8_local_physical_control_screen",
            "taoryx.b747_local_physical_control_screen",
            "taoryx_source_table_fixed_wing.episode",
        ),
        adapter_builds=(
            ("skywalker_x8", "rigid_body_6dof_direct_wrench"),
            ("b747", "rigid_body_6dof_direct_wrench"),
        ),
        resource_module="taoryx_source_table_fixed_wing.resources",
        composition_witness="examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml",
        additional_composition_witnesses=("examples/vehicle_composition/b747_racetrack_capability_3dof_compose.yaml",),
        exercise_language_backed_materialization=True,
        focused_provider_models=(
            ("taoryx.x8.mission-composition", ("skywalker_x8",)),
            ("taoryx.b747.mission-composition", ("b747",)),
        ),
    ),
    PluginWheelSpec(
        selector="x15",
        project=ROOT / "packages" / "taoryx-x15",
        wheel_stem="taoryx_x15",
        plugin_id="taoryx.x15",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.x15_adapter",
            "taoryx.x15_local_direct_wrench",
            "taoryx.x15_local_physical_surface_lqi_screen",
            "taoryx.x15_maneuvers",
            "taoryx.x15_surface_authority_screen",
        ),
        deferred_module_names=(
            "numpy",
            "scipy",
            "taoryx.aero_drag_tables",
            "taoryx.control_allocation",
            "taoryx.direct_wrench",
            "taoryx.family_adapter",
            "taoryx.family_adapter_registry",
            "taoryx.local_direct_wrench",
            "taoryx.local_direct_wrench_batch_episode_parity",
            "taoryx.local_direct_wrench_composition_execution",
            "taoryx.local_direct_wrench_mission_translation",
            "taoryx.physical_lqr",
            "taoryx.x15_adapter",
            "taoryx.x15_local_physical_surface_lqi_screen",
            "taoryx.x15_maneuvers",
            "taoryx.x15_surface_authority_screen",
        ),
        adapter_builds=(("x15", "rigid_body_6dof_direct_wrench"),),
        resource_module="taoryx_x15.resources",
        composition_witness="examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml",
        batch_only_witnesses=(
            "examples/vehicle_composition/x15_local_direct_wrench_lqi_screen_compose.yaml",
            "examples/vehicle_composition/x15_source_surface_authority_screen_compose.yaml",
            "examples/vehicle_composition/x15_source_surface_attitude_rate_lqi_screen_compose.yaml",
        ),
        focused_provider_id="taoryx.x15.mission-composition",
        excluded_mission_template_ids=("x15_staged_booster_reachability_v1",),
        exercise_episode=False,
    ),
    PluginWheelSpec(
        selector="hl20",
        project=ROOT / "packages" / "taoryx-hl20",
        wheel_stem="taoryx_hl20",
        plugin_id="taoryx.hl20",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.hl20_adapter",
            "taoryx.hl20_controls",
            "taoryx.hl20_local_direct_wrench",
            "taoryx.hl20_local_physical_surface_lqi_screen",
            "taoryx.hl20_source_aerodynamics",
            "taoryx.hl20_source_model",
            "taoryx.hl20_surface_authority_screen",
            "taoryx.hl20_trim_evidence",
        ),
        adapter_builds=(("hl20_mod_k", "rigid_body_6dof_direct_wrench"),),
        resource_module="taoryx_hl20.resources",
        composition_witness="examples/vehicle_composition/hl20_local_direct_wrench_screen_compose.yaml",
        focused_provider_id="taoryx.hl20.mission-composition",
        excluded_mission_template_ids=(
            "hl20_source_booster_release_replay_v1",
            "lifting_body_glide_energy_management_v1",
        ),
        trim_evidence_family_id="reference_hl20_mod_k",
        exercise_episode=False,
        exercise_composition_batch=False,
        wheel_dependency_selectors=("daveml",),
    ),
    PluginWheelSpec(
        selector="reachability",
        project=ROOT / "packages" / "taoryx-reachability",
        wheel_stem="taoryx_reachability",
        plugin_id="taoryx.reachability",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx_reachability.plugin",
            "taoryx_reachability.provider",
            "taoryx_reachability.resources",
        ),
        deferred_module_names=(
            "taoryx_reachability.capabilities",
            "taoryx_reachability.preflight",
            "taoryx.hl20_source_aerodynamics",
            "taoryx.hl20_source_model",
            "taoryx.hl20_source_release_composition_execution",
            "taoryx.x15_staged_composition_execution",
        ),
        exercise_episode=False,
        exercise_provider_catalog=False,
        wheel_dependency_selectors=("daveml", "x15", "hl20"),
        include_by_default=False,
        forbidden_wheel_prefixes=(
            "taoryx/passive_tumbling_composition_execution.py",
            "taoryx/passive_tumbling_mission_translation.py",
        ),
        required_wheel_paths=(
            "taoryx/reachability_catalog.py",
            "taoryx/reachability_envelope.py",
            "taoryx_reachability/capabilities.py",
            "taoryx_reachability/preflight.py",
            "taoryx_reachability/provider.py",
            "taoryx_reachability/data/package_data_provenance.json",
            "taoryx_reachability/data/verification/reachability_profile_catalog.yaml",
            "taoryx_reachability/data/x15_overlay/verification/vehicle_composition_registry.yaml",
            "taoryx_reachability/data/x15_overlay/verification/vehicle_execution_bindings.yaml",
            "taoryx_reachability/data/x15_overlay/verification/vehicle_execution_witnesses.yaml",
            "taoryx_reachability/data/x15_overlay/examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
            "taoryx_reachability/data/hl20_overlay/verification/vehicle_composition_registry.yaml",
            "taoryx_reachability/data/hl20_overlay/verification/vehicle_execution_bindings.yaml",
            "taoryx_reachability/data/hl20_overlay/verification/vehicle_execution_witnesses.yaml",
            "taoryx_reachability/data/hl20_overlay/examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
            "taoryx_reachability/data/hl20_overlay/examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
            "taoryx_reachability/data/examples/showcases/hl20_california_to_hawaii/quality_gates.yaml",
        ),
    ),
    PluginWheelSpec(
        selector="debug-models",
        project=ROOT / "packages" / "taoryx-debug-models",
        wheel_stem="taoryx_debug_models",
        plugin_id="taoryx.debug-models",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.trajectory.contract_probe_mission_composition",
            "taoryx.trajectory.reference_mission_composition",
        ),
        workflow_endpoint_ids=(
            "reference-ballistic-3dof-batch",
            "reference-waypoint-3dof-batch",
            "debug-contract-probe-batch",
        ),
    ),
    PluginWheelSpec(
        selector="simple-aero",
        project=ROOT / "packages" / "taoryx-simple-aero",
        wheel_stem="taoryx_simple_aero",
        plugin_id="taoryx.simple-aero",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx_simple_aero.mission_composition",
            "taoryx_simple_aero.provider",
            "taoryx.simple_aero_builder",
            "taoryx.trajectory.simple_aero_mission_composition",
        ),
        focused_provider_models=(("taoryx.simple-aero.mission-composition", ("simple_aero",)),),
        exercise_episode=False,
        workflow_endpoint_ids=("simple-aero-fixed-ld-batch",),
    ),
    PluginWheelSpec(
        selector="dual-launch",
        project=ROOT / "packages" / "taoryx-dual-launch",
        wheel_stem="taoryx_dual_launch",
        plugin_id="taoryx.dual-launch",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx_dual_launch.mission_composition",
            "taoryx.trajectory.dual_launch",
            "taoryx.trajectory.dual_launch_mission_composition",
        ),
        focused_provider_models=(("taoryx.dual-launch.mission-composition", ("dual_launch_glider",)),),
        exercise_episode=False,
        workflow_endpoint_ids=("dual-launch-attached-booster-batch",),
    ),
    PluginWheelSpec(
        selector="reference-models",
        project=ROOT / "packages" / "taoryx-reference-models",
        wheel_stem="taoryx_reference_models",
        plugin_id="taoryx.reference-models",
        plugin_version="0.1.0a0",
        owned_modules=(),
        exercise_episode=False,
        exercise_provider_catalog=False,
        forbidden_wheel_prefixes=("taoryx/",),
        forbidden_top_level_packages=("taoryx",),
        include_by_default=False,
    ),
    PluginWheelSpec(
        selector="nesc",
        project=ROOT / "packages" / "taoryx-nesc",
        wheel_stem="taoryx_nesc",
        plugin_id="taoryx.nesc",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.nesc_adapter",
            "taoryx.nesc_composition_execution",
            "taoryx.nesc_mission_translation",
            "taoryx.trajectory.nesc_pseudo6dof",
        ),
        adapter_builds=(
            ("reference_nesc_two_stage_rocket", "point_mass_3dof"),
            ("reference_nesc_two_stage_rocket", "pseudo_6dof"),
        ),
        resource_module="taoryx_nesc.resources",
        composition_witness="examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml",
        exercise_episode=False,
        wheel_dependency_selectors=("daveml",),
    ),
    PluginWheelSpec(
        selector="passive-bodies",
        project=ROOT / "packages" / "taoryx-passive-bodies",
        wheel_stem="taoryx_passive_bodies",
        plugin_id="taoryx.passive-bodies",
        plugin_version="0.1.0a0",
        owned_modules=(
            "taoryx.passive_body_dynamics",
            "taoryx.passive_tumbling_composition_execution",
            "taoryx_passive_bodies.deployment_runtime",
        ),
        deferred_module_names=(
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
        ),
        adapter_builds=(
            ("tumbling_body", "point_mass_3dof"),
            ("tumbling_body", "pseudo_6dof"),
        ),
        resource_module="taoryx_passive_bodies.resources",
        composition_witness="examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml",
        exercise_episode=False,
    ),
    PluginWheelSpec(
        selector="cross-plugin-deployment",
        project=ROOT / "packages" / "taoryx-nesc",
        wheel_stem="taoryx_nesc",
        plugin_id="taoryx.nesc",
        plugin_version="0.1.0a0",
        owned_modules=("taoryx.nesc_composition_execution",),
        adapter_builds=(("reference_nesc_two_stage_rocket", "pseudo_6dof"),),
        resource_module="taoryx_nesc.resources",
        composition_witness="examples/vehicle_composition/nesc_staged_source_replay_with_passive_child_pseudo6dof_compose.yaml",
        exercise_episode=False,
        wheel_dependency_selectors=("daveml", "passive-bodies"),
        include_by_default=False,
        focused_provider_id="taoryx.nesc.mission-composition",
        deployment_child_runtime_id="taoryx.passive-bodies.local-atmosphere-release.v1",
    ),
)

_SMOKE_SCRIPT = r"""
from __future__ import annotations

import importlib
import json
import os
import sys
from tempfile import TemporaryDirectory
from importlib.metadata import distributions
from pathlib import Path

target = Path(os.environ["TAORYX_WHEEL_TARGET"]).resolve()
source_roots = tuple(Path(item).resolve() for item in json.loads(os.environ["TAORYX_WHEEL_SOURCE_ROOTS"]))
expected = json.loads(os.environ["TAORYX_WHEEL_EXPECTED"])

# Preserve the active environment's third-party dependencies, but remove every
# editable Taoryx source root before the installed wheel imports.  The process
# starts in a temporary directory, so an empty sys.path entry cannot point at
# the repository either.
sys.path[:] = [
    item
    for item in sys.path
    if not item or not any(Path(item).resolve().is_relative_to(source_root) for source_root in source_roots)
]
sys.path.insert(0, str(target))

entries = tuple(
    entry
    for distribution in distributions(path=[str(target)])
    for entry in distribution.entry_points
    if entry.group == "taoryx.plugins"
)
entry_ids = {entry.name for entry in entries}
expected_ids = {item["plugin_id"] for item in expected}
if entry_ids != expected_ids:
    raise RuntimeError(f"wheel entry points were {sorted(entry_ids)!r}, expected {sorted(expected_ids)!r}")

from taoryx.plugins.discovery import discover_plugins

catalog = discover_plugins(include_builtin=False, entry_points=entries)
catalog_ids = {plugin.id for plugin in catalog.plugins}
if catalog_ids != expected_ids:
    raise RuntimeError(f"wheel catalog was {sorted(catalog_ids)!r}, expected {sorted(expected_ids)!r}")
for item in expected:
    if not item["exercise_boundary"]:
        continue
    deferred_module_names = item["deferred_module_names"]
    imported = tuple(module_name for module_name in deferred_module_names if module_name in sys.modules)
    if imported:
        raise RuntimeError(
            f"wheel plug-in {item['plugin_id']!r} imported deferred implementation modules during discovery: {imported!r}"
        )

for item in expected:
    revision = catalog.plugin_revision(item["plugin_id"])
    if revision.version != item["plugin_version"]:
        raise RuntimeError(
            f"wheel plug-in {item['plugin_id']!r} advertised version {revision.version!r}, "
            f"expected {item['plugin_version']!r}"
        )
    if len(revision.fingerprint) != 64:
        raise RuntimeError(f"wheel plug-in {item['plugin_id']!r} has an invalid revision fingerprint")
    if not item["exercise_boundary"]:
        continue
    for format_id, module_name in item["model_format_imports"]:
        contribution = catalog.contribution("model_format", format_id)
        if contribution.plugin.id != item["plugin_id"]:
            raise RuntimeError(
                f"wheel model format {format_id!r} is owned by {contribution.plugin.id!r}, "
                f"not {item['plugin_id']!r}"
            )
        importer = getattr(contribution.value, "import_module", None)
        if not callable(importer):
            raise RuntimeError(f"wheel model format {format_id!r} does not expose a callable lazy importer")
        module = importer()
        if module.__name__ != module_name:
            raise RuntimeError(
                f"wheel model format {format_id!r} imported {module.__name__!r}, expected {module_name!r}"
            )
        module_file = Path(module.__file__).resolve()
        if not module_file.is_relative_to(target):
            raise RuntimeError(f"wheel model-format module {module_name!r} came from {module_file}, not {target}")

reachability_item = next((item for item in expected if item["plugin_id"] == "taoryx.reachability"), None)
if reachability_item is not None and reachability_item["exercise_boundary"]:
    from taoryx.vehicle_catalog_resources import vehicle_catalog_resource, vehicle_catalog_resources
    from taoryx.vehicle_composition_registry import load_resolved_vehicle_composition_catalog
    from taoryx.vehicle_execution_bindings import load_vehicle_execution_binding_catalog
    from taoryx.vehicle_execution_witnesses import load_vehicle_execution_witness_catalog

    overlay_only = discover_plugins(
        include_builtin=False,
        entry_points=entries,
        selected=("taoryx.reachability",),
    )
    try:
        vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=overlay_only)
    except FileNotFoundError as error:
        if "requirements are inactive" not in str(error):
            raise RuntimeError(f"wheel reachability overlay failed with an unexpected diagnostic: {error}") from error
    else:
        raise RuntimeError("wheel reachability overlay resolved without a required vehicle base fragment")

    overlay_catalog = discover_plugins(
        include_builtin=False,
        entry_points=entries,
        selected=("taoryx.x15", "taoryx.reachability"),
    )
    overlay_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=overlay_catalog)
    if len(overlay_resources) != 2 or not all(path.resolve().is_relative_to(target) for path in overlay_resources):
        raise RuntimeError(f"wheel reachability overlay did not resolve exactly two installed catalog fragments: {overlay_resources!r}")
    x15 = load_resolved_vehicle_composition_catalog(plugins=overlay_catalog).vehicle("x15")
    mission_ids = {mission.id for mission in x15.declaration.mission_templates}
    if "x15_staged_booster_reachability_v1" not in mission_ids:
        raise RuntimeError("wheel reachability overlay did not add the staged X-15 mission")
    staged_bindings = {
        (binding.mission, binding.fidelity)
        for binding in load_vehicle_execution_binding_catalog(plugins=overlay_catalog).bindings
        if binding.mission == "x15_staged_booster_reachability_v1"
    }
    if staged_bindings != {
        ("x15_staged_booster_reachability_v1", "point_mass_3dof"),
        ("x15_staged_booster_reachability_v1", "pseudo_6dof"),
    }:
        raise RuntimeError(f"wheel reachability overlay staged bindings were {sorted(staged_bindings)!r}")
    witness_ids = {witness.id for witness in load_vehicle_execution_witness_catalog(plugins=overlay_catalog).witnesses}
    if not {"x15-staged-3dof-batch", "x15-staged-pseudo6dof-batch"} <= witness_ids:
        raise RuntimeError(f"wheel reachability overlay staged witnesses were {sorted(witness_ids)!r}")
    staged_request = vehicle_catalog_resource(
        "examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
        plugins=overlay_catalog,
    )
    if not staged_request.resolve().is_relative_to(target / "taoryx_reachability" / "data" / "x15_overlay"):
        raise RuntimeError(f"wheel reachability overlay request came from the wrong package: {staged_request}")

    hl20_scope = discover_plugins(
        include_builtin=False,
        entry_points=entries,
        selected=("taoryx.hl20", "taoryx.reachability"),
    )
    hl20_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=hl20_scope)
    if len(hl20_resources) != 2 or not all(path.resolve().is_relative_to(target) for path in hl20_resources):
        raise RuntimeError(f"wheel reachability HL-20 overlay did not resolve exactly two installed catalog fragments: {hl20_resources!r}")
    if "hl20_overlay" not in hl20_resources[1].parts:
        raise RuntimeError(f"wheel reachability HL-20 overlay came from the wrong resource root: {hl20_resources!r}")
    hl20 = load_resolved_vehicle_composition_catalog(plugins=hl20_scope).vehicle("hl20_mod_k")
    hl20_missions = {mission.id for mission in hl20.declaration.mission_templates}
    expected_hl20_missions = {
        "hl20_source_booster_release_replay_v1",
        "lifting_body_glide_energy_management_v1",
    }
    if not expected_hl20_missions <= hl20_missions:
        raise RuntimeError(f"wheel reachability HL-20 overlay missions were {sorted(hl20_missions)!r}")
    hl20_bindings = {
        (binding.mission, binding.fidelity)
        for binding in load_vehicle_execution_binding_catalog(plugins=hl20_scope).bindings
        if binding.mission in expected_hl20_missions
    }
    if hl20_bindings != {
        ("hl20_source_booster_release_replay_v1", "point_mass_3dof"),
        ("hl20_source_booster_release_replay_v1", "pseudo_6dof"),
        ("lifting_body_glide_energy_management_v1", "rigid_body_6dof_direct_wrench"),
        ("lifting_body_glide_energy_management_v1", "rigid_body_6dof_surface_allocated"),
    }:
        raise RuntimeError(f"wheel reachability HL-20 bindings were {sorted(hl20_bindings)!r}")
    hl20_witness_ids = {witness.id for witness in load_vehicle_execution_witness_catalog(plugins=hl20_scope).witnesses}
    if not {"hl20-source-release-3dof-batch", "hl20-source-release-pseudo6dof-batch"} <= hl20_witness_ids:
        raise RuntimeError(f"wheel reachability HL-20 witnesses were {sorted(hl20_witness_ids)!r}")
    for request_name in (
        "examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
        "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
    ):
        request = vehicle_catalog_resource(request_name, plugins=hl20_scope)
        if not request.resolve().is_relative_to(target / "taoryx_reachability" / "data" / "hl20_overlay"):
            raise RuntimeError(f"wheel reachability HL-20 request came from the wrong package: {request}")

    combined_scope = discover_plugins(
        include_builtin=False,
        entry_points=entries,
        selected=("taoryx.x15", "taoryx.hl20", "taoryx.reachability"),
    )
    combined_resources = vehicle_catalog_resources("verification/vehicle_composition_registry.yaml", plugins=combined_scope)
    if len(combined_resources) != 4 or sum("x15_overlay" in path.parts for path in combined_resources) != 1 or sum(
        "hl20_overlay" in path.parts for path in combined_resources
    ) != 1:
        raise RuntimeError(f"wheel reachability overlays did not remain isolated when selected together: {combined_resources!r}")
    combined_catalog = load_resolved_vehicle_composition_catalog(plugins=combined_scope)
    if "x15_staged_booster_reachability_v1" not in {
        mission.id for mission in combined_catalog.vehicle("x15").declaration.mission_templates
    } or not expected_hl20_missions <= {
        mission.id for mission in combined_catalog.vehicle("hl20_mod_k").declaration.mission_templates
    }:
        raise RuntimeError("wheel reachability overlays did not remain active for their own bases when selected together")

provider_items = tuple(item for item in expected if item["exercise_boundary"] and item["exercise_provider_catalog"])
providers = catalog.build_mission_composition_provider_registry() if provider_items else None
if providers is not None:
    provider_plugin_ids = {item["plugin_id"] for item in provider_items}
    for contribution in catalog.records("mission_composition_provider"):
        if contribution.plugin.id not in provider_plugin_ids:
            continue
        provider = providers.provider(contribution.id)
        models = provider.list_models()
        if not models:
            raise RuntimeError(f"wheel provider {provider.metadata.id!r} exposes no models")
        for model in models:
            if not model.version or len(model.metadata_fingerprint) != 64:
                raise RuntimeError(f"wheel model {model.id!r} has an invalid version or metadata revision")
            provider.get_model_schema(model.id)
    for item in provider_items:
        provider_id = item["focused_provider_id"]
        excluded_mission_template_ids = set(item["excluded_mission_template_ids"])
        for focused_provider_id, expected_model_ids in item["focused_provider_models"]:
            provider = providers.provider(focused_provider_id)
            actual_model_ids = tuple(model.id for model in provider.list_models())
            if actual_model_ids != tuple(expected_model_ids):
                raise RuntimeError(
                    f"wheel provider {focused_provider_id!r} exposed {actual_model_ids!r}, "
                    f"expected {tuple(expected_model_ids)!r}"
                )
        if provider_id is None or not excluded_mission_template_ids:
            continue
        provider = providers.provider(provider_id)
        for model in provider.list_models():
            exposed_mission_template_ids = {template.id for template in model.mission_templates}
            leaked = sorted(exposed_mission_template_ids & excluded_mission_template_ids)
            if leaked:
                raise RuntimeError(
                    f"wheel provider {provider_id!r} exposed missions owned by an optional overlay: {leaked!r}"
                )

adapter_registry = None
for item in expected:
    if not item["exercise_boundary"]:
        continue
    for family_id, tier in item["adapter_builds"]:
        if adapter_registry is None:
            adapter_registry = catalog.build_family_adapter_registry()
        adapter = adapter_registry.build(family_id, tier)
        if adapter.describe().family_id != family_id:
            raise RuntimeError(f"wheel adapter for {family_id!r} returned a mismatched family")
    for module_name in item["owned_modules"]:
        module = importlib.import_module(module_name)
        module_file = Path(module.__file__).resolve()
        if not module_file.is_relative_to(target):
            raise RuntimeError(f"wheel module {module_name!r} came from {module_file}, not {target}")
    trim_evidence_family_id = item["trim_evidence_family_id"]
    if trim_evidence_family_id is not None:
        from taoryx.vehicle_trim_adapters import solve_vehicle_trim_evidence

        trim_report = solve_vehicle_trim_evidence(trim_evidence_family_id, plugins=catalog)
        if trim_report.status != "verified":
            raise RuntimeError(
                f"wheel trim evidence did not verify {trim_evidence_family_id!r}: {trim_report.findings!r}"
            )
    resource_module_name = item["resource_module"]
    composition_witness = item["composition_witness"]
    additional_composition_witnesses = item["additional_composition_witnesses"]
    batch_only_witnesses = item["batch_only_witnesses"]
    exercise_episode = item["exercise_episode"]
    exercise_composition_batch = item["exercise_composition_batch"]
    exercise_language_backed_materialization = item["exercise_language_backed_materialization"]
    deployment_child_runtime_id = item["deployment_child_runtime_id"]
    workflow_endpoint_ids = item["workflow_endpoint_ids"]
    if workflow_endpoint_ids:
        from taoryx.mission_workflow_endpoint import verify_mission_workflow_endpoint

        for workflow_endpoint_id in workflow_endpoint_ids:
            workflow_report = verify_mission_workflow_endpoint(
                workflow_endpoint_id,
                execute=True,
                plugins=catalog,
            )
            if workflow_report["status"] != "pass":
                raise RuntimeError(
                    f"wheel workflow endpoint {workflow_endpoint_id!r} did not pass: {workflow_report!r}"
                )
    if resource_module_name is not None or composition_witness is not None:
        if resource_module_name is None or composition_witness is None:
            raise RuntimeError(f"wheel plug-in {item['plugin_id']!r} declares an incomplete composition witness")
        resource_module = importlib.import_module(resource_module_name)
        resource_root = getattr(resource_module, "model_resource_root")()
        witness_path = Path(resource_root) / composition_witness
        if not witness_path.is_file():
            raise RuntimeError(f"wheel composition witness is not packaged: {witness_path}")
        from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
        from taoryx.vehicle_composition import compile_vehicle_composition, load_vehicle_composition_request

        source_request = load_vehicle_composition_request(witness_path)
        composition = compile_vehicle_composition(source_request, plugins=catalog)
        # A family can assign execution evidence to its focused vertical gate
        # while this wheel smoke still proves the installed package owns and
        # compiles its declared composition without a source fallback.
        if not exercise_composition_batch:
            continue
        if exercise_language_backed_materialization:
            from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition

            with TemporaryDirectory(prefix="taoryx-wheel-language-backed-materialization-") as materialization_directory:
                materialized = materialize_powered_fixed_wing_composition(
                    composition,
                    Path(materialization_directory),
                    plugins=catalog,
                )
            if not materialized.asset_root.resolve().is_relative_to(target):
                raise RuntimeError(
                    "wheel language-backed materialization did not resolve its package-owned route assets: "
                    f"{materialized.asset_root}"
                )
        with TemporaryDirectory(prefix="taoryx-wheel-composition-") as output_directory:
            batch = execute_vehicle_composition_batch(composition, output_directory, plugins=catalog)
        if not batch.passed:
            raise RuntimeError(f"wheel composition batch did not pass: {composition.id!r}")
        if deployment_child_runtime_id is not None:
            payload = batch.execution.as_dict()
            children = payload.get("deployment_children")
            if not isinstance(children, list) or len(children) != 1:
                raise RuntimeError(f"wheel composition did not emit one selected child: {composition.id!r}")
            child = children[0]
            if not isinstance(child, dict) or child.get("runtime_id") != deployment_child_runtime_id:
                raise RuntimeError(
                    f"wheel composition child runtime was {child.get('runtime_id') if isinstance(child, dict) else child!r}, "
                    f"expected {deployment_child_runtime_id!r}"
                )
            from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection, MissionCompositionRunRequest
            from taoryx.trajectory.native_mission_composition import (
                build_registry_mission_composition_runner,
                configuration_instance_from_vehicle_request,
            )

            provider_id = item["focused_provider_id"]
            if providers is None or provider_id is None:
                raise RuntimeError(
                    "wheel cross-plugin deployment proof requires its selected mission composition provider"
                )
            api_provider = providers.provider(provider_id)
            parent_request = source_request.model_copy(update={"deployment_bindings": ()})
            configuration = configuration_instance_from_vehicle_request(api_provider, parent_request)
            prepared = api_provider.validate_configuration(configuration)
            api_request = MissionCompositionRunRequest(
                request_id="installed-wheel-cross-plugin-deployment",
                provider_id=api_provider.metadata.id,
                provider_version=api_provider.metadata.version,
                prepared_configuration=prepared,
                deployment_bindings=source_request.deployment_bindings,
                output=MissionCompositionOutputSelection(mode="core"),
            )
            api_response = build_registry_mission_composition_runner(api_provider).run(api_request)
            if api_response.kind != "trajectory" or len(api_response.result.relationships) != 1:
                raise RuntimeError("installed-wheel common Mission Composition API did not retain the selected child")
        if exercise_episode:
            from taoryx.composition_episode import open_vehicle_composition_episode

            episode = open_vehicle_composition_episode(composition, plugins=catalog)
            try:
                if episode.__class__.__module__ not in item["owned_modules"]:
                    raise RuntimeError(
                        f"wheel composition episode came from {episode.__class__.__module__!r}, "
                        f"not one of {item['owned_modules']!r}"
                    )
            finally:
                episode.close()
        for additional_composition_witness in additional_composition_witnesses:
            additional_witness_path = Path(resource_root) / additional_composition_witness
            if not additional_witness_path.is_file():
                raise RuntimeError(f"wheel additional composition witness is not packaged: {additional_witness_path}")
            additional_request = load_vehicle_composition_request(additional_witness_path)
            additional_composition = compile_vehicle_composition(additional_request, plugins=catalog)
            with TemporaryDirectory(prefix="taoryx-wheel-additional-composition-") as output_directory:
                additional_batch = execute_vehicle_composition_batch(additional_composition, output_directory, plugins=catalog)
            if not additional_batch.passed:
                raise RuntimeError(f"wheel additional composition batch did not pass: {additional_composition.id!r}")
            if exercise_episode:
                additional_episode = open_vehicle_composition_episode(additional_composition, plugins=catalog)
                try:
                    if additional_episode.__class__.__module__ not in item["owned_modules"]:
                        raise RuntimeError(
                            f"wheel additional composition episode came from {additional_episode.__class__.__module__!r}, "
                            f"not one of {item['owned_modules']!r}"
                        )
                finally:
                    additional_episode.close()
        for batch_only_witness in batch_only_witnesses:
            batch_only_path = Path(resource_root) / batch_only_witness
            if not batch_only_path.is_file():
                raise RuntimeError(f"wheel batch-only composition witness is not packaged: {batch_only_path}")
            batch_only_request = load_vehicle_composition_request(batch_only_path)
            batch_only_composition = compile_vehicle_composition(batch_only_request, plugins=catalog)
            with TemporaryDirectory(prefix="taoryx-wheel-batch-only-") as output_directory:
                batch_only = execute_vehicle_composition_batch(batch_only_composition, output_directory, plugins=catalog)
            if not batch_only.passed:
                raise RuntimeError(f"wheel batch-only composition did not pass: {batch_only_composition.id!r}")

taoryx_module = importlib.import_module("taoryx")
if not Path(taoryx_module.__file__).resolve().is_relative_to(target):
    raise RuntimeError("the core Taoryx module did not load from the installed wheel target")
if "taoryx.reference-models" not in expected_ids and "taoryx_reference_models" in sys.modules:
    raise RuntimeError("wheel smoke unexpectedly imported the reference-model aggregate")

print("Installed-wheel plug-in smoke passed:", ", ".join(sorted(expected_ids)))
"""


def parse_args() -> argparse.Namespace:
    """Parse the focused wheel set to build and install."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plugin",
        action="append",
        choices=tuple(item.selector for item in PLUGIN_WHEEL_SPECS),
        help="plug-in wheel to verify; repeat to select a subset (default: all isolated plug-ins)",
    )
    parser.add_argument(
        "--python",
        default=sys.executable,
        help="Python interpreter with build and test dependencies (default: current interpreter)",
    )
    return parser.parse_args()
    ####


def requested_selectors(selectors: Iterable[str] | None) -> frozenset[str]:
    """Choose the plug-in boundaries a caller explicitly wants to exercise."""

    requested = frozenset(selectors or ())
    if requested:
        return requested
    return frozenset(spec.selector for spec in PLUGIN_WHEEL_SPECS if spec.include_by_default)
    ####


def selected_specs(selectors: Iterable[str] | None) -> tuple[PluginWheelSpec, ...]:
    """Resolve requested wheel boundaries and their install-only dependencies."""

    selected = requested_selectors(selectors)
    by_selector = {spec.selector: spec for spec in PLUGIN_WHEEL_SPECS}
    expanded = set(selected)
    pending = list(selected)
    while pending:
        selector = pending.pop()
        for dependency in by_selector[selector].wheel_dependency_selectors:
            if dependency not in expanded:
                expanded.add(dependency)
                pending.append(dependency)
    return tuple(spec for spec in PLUGIN_WHEEL_SPECS if spec.selector in expanded)
    ####


def smoke_expectations(
    specs: tuple[PluginWheelSpec, ...],
    *,
    exercised_selectors: frozenset[str],
) -> list[dict[str, object]]:
    """Serialize installed-wheel assertions without exercising dependency internals."""

    return [
        {
            "plugin_id": spec.plugin_id,
            "plugin_version": spec.plugin_version,
            "exercise_boundary": spec.selector in exercised_selectors,
            "deferred_module_names": spec.deferred_module_names,
            "owned_modules": spec.owned_modules,
            "model_format_imports": spec.model_format_imports,
            "adapter_builds": spec.adapter_builds,
            "resource_module": spec.resource_module,
            "composition_witness": spec.composition_witness,
            "additional_composition_witnesses": spec.additional_composition_witnesses,
            "batch_only_witnesses": spec.batch_only_witnesses,
            "focused_provider_id": spec.focused_provider_id,
            "focused_provider_models": spec.focused_provider_models,
            "excluded_mission_template_ids": spec.excluded_mission_template_ids,
            "trim_evidence_family_id": spec.trim_evidence_family_id,
            "exercise_episode": spec.exercise_episode,
            "exercise_composition_batch": spec.exercise_composition_batch,
            "exercise_language_backed_materialization": spec.exercise_language_backed_materialization,
            "deployment_child_runtime_id": spec.deployment_child_runtime_id,
            "workflow_endpoint_ids": spec.workflow_endpoint_ids,
            "exercise_provider_catalog": spec.exercise_provider_catalog,
        }
        for spec in specs
    ]
    ####


def clean_environment() -> dict[str, str]:
    """Prevent developer-source ``PYTHONPATH`` entries from masking wheel bugs."""

    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    return environment
    ####


def resolve_python(python: str) -> str:
    """Keep a PATH command intact, but stabilize a relative interpreter path."""

    if os.sep in python or (os.altsep is not None and os.altsep in python):
        return str(Path(python).resolve())
    return python
    ####


def command_display(command: list[str]) -> str:
    """Keep an inline child script from overwhelming normal smoke output."""

    if len(command) == 3 and command[1] == "-c" and command[2] == _SMOKE_SCRIPT:
        return f"{command[0]} -c <installed-wheel smoke>"
    return " ".join(command)
    ####


def run_command(
    command: list[str],
    *,
    cwd: Path,
    environment: dict[str, str],
    show_stdout: bool = False,
) -> None:
    """Run one child command quietly on success and replay diagnostics on failure."""

    print("+", command_display(command))
    result = subprocess.run(command, cwd=cwd, env=environment, text=True, capture_output=True, check=False)
    if result.returncode == 0:
        if show_stdout and result.stdout:
            print(result.stdout, end="")
        return
    if result.stdout:
        print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)
    raise subprocess.CalledProcessError(result.returncode, command)
    ####


def wheel_for(wheelhouse: Path, stem: str) -> Path:
    """Find the one fresh wheel emitted for an intended distribution."""

    candidates = tuple(sorted(wheelhouse.glob(f"{stem}-*.whl")))
    if len(candidates) != 1:
        raise FileNotFoundError(f"expected one {stem!r} wheel in {wheelhouse}, found {len(candidates)}")
    return candidates[0]
    ####


def verify_wheel_layout(wheel: Path, spec: PluginWheelSpec) -> None:
    """Reject excluded namespace payloads before an installed smoke can mask them."""

    if not spec.forbidden_wheel_prefixes and not spec.forbidden_top_level_packages and not spec.required_wheel_paths:
        return
    with zipfile.ZipFile(wheel) as archive:
        names = frozenset(archive.namelist())
        leaked = tuple(name for name in names if any(name.startswith(prefix) for prefix in spec.forbidden_wheel_prefixes))
    if leaked:
        raise RuntimeError(f"wheel {wheel.name!r} contains excluded package payloads for {spec.selector!r}: {leaked!r}")

    missing = tuple(path for path in spec.required_wheel_paths if path not in names)
    if missing:
        raise RuntimeError(f"wheel {wheel.name!r} is missing required package payloads for {spec.selector!r}: {missing!r}")

    if not spec.forbidden_top_level_packages:
        return
    with zipfile.ZipFile(wheel) as archive:
        top_level_files = tuple(name for name in archive.namelist() if name.endswith(".dist-info/top_level.txt"))
        if len(top_level_files) != 1:
            raise RuntimeError(f"wheel {wheel.name!r} must contain exactly one top-level package manifest for {spec.selector!r}; found {top_level_files!r}")
        top_level_packages = frozenset(item.strip() for item in archive.read(top_level_files[0]).decode("utf-8").splitlines() if item.strip())
    leaked_top_levels = tuple(sorted(top_level_packages & frozenset(spec.forbidden_top_level_packages)))
    if leaked_top_levels:
        raise RuntimeError(f"wheel {wheel.name!r} advertises excluded top-level packages for {spec.selector!r}: {leaked_top_levels!r}")
    ####


def build_and_smoke(
    *,
    python: str,
    specs: tuple[PluginWheelSpec, ...],
    exercised_selectors: frozenset[str] | None = None,
) -> None:
    """Build fresh wheels and exercise only the requested installed boundaries."""

    environment = clean_environment()
    selected_boundaries = frozenset(spec.selector for spec in specs) if exercised_selectors is None else exercised_selectors
    with tempfile.TemporaryDirectory(prefix="taoryx-plugin-wheel-smoke-") as temporary_directory:
        temporary_root = Path(temporary_directory)
        wheelhouse = temporary_root / "wheelhouse"
        target = temporary_root / "target"
        wheelhouse.mkdir()

        # Do not use ``--wheel`` here.  Building the wheel from the sdist gives
        # setuptools a new build tree, so stale files in this checkout's build/
        # directory cannot be carried into an artifact.
        build_command = [python, "-m", "build", "--no-isolation", "--outdir", str(wheelhouse)]
        run_command([*build_command, str(ROOT)], cwd=temporary_root, environment=environment)
        for spec in specs:
            run_command([*build_command, str(spec.project)], cwd=temporary_root, environment=environment)

        plugin_wheels = {spec.selector: wheel_for(wheelhouse, spec.wheel_stem) for spec in specs}
        for spec in specs:
            verify_wheel_layout(plugin_wheels[spec.selector], spec)
        wheels = [wheel_for(wheelhouse, "taoryx"), *(plugin_wheels[spec.selector] for spec in specs)]
        run_command(
            [python, "-m", "pip", "install", "--no-deps", "--target", str(target), *(str(item) for item in wheels)],
            cwd=temporary_root,
            environment=environment,
        )
        smoke_environment = environment | {
            "TAORYX_WHEEL_TARGET": str(target),
            "TAORYX_WHEEL_SOURCE_ROOTS": json.dumps([str(ROOT / "src"), str(ROOT / "packages")]),
            "TAORYX_WHEEL_EXPECTED": json.dumps(smoke_expectations(specs, exercised_selectors=selected_boundaries)),
        }
        run_command(
            [python, "-c", _SMOKE_SCRIPT],
            cwd=temporary_root,
            environment=smoke_environment,
            show_stdout=True,
        )
    ####


def main() -> int:
    """Run the selected installed-wheel plug-in smoke path."""

    args = parse_args()
    boundaries = requested_selectors(args.plugin)
    specs = selected_specs(boundaries)
    build_and_smoke(python=resolve_python(args.python), specs=specs, exercised_selectors=boundaries)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
