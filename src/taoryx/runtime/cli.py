"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import base64
import json
import re
import shlex
from collections.abc import Mapping
from html import escape
from importlib import metadata as importlib_metadata
from pathlib import Path
from typing import Any, TypedDict, cast

from taoryx.composition_result_catalog import index_composition_results, write_composition_release_catalog
from taoryx.controller_tuning_registry import (
    ControllerTuningCampaignRegistration,
    ControllerTuningCampaignRegistry,
)
from taoryx.integration import available_integrator_descriptions, available_integrators
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition
from taoryx.mission_workflow_endpoint import (
    mission_workflow_endpoint_list,
    verify_mission_workflow_endpoint,
)
from taoryx.model_authoring import (
    ModelAuthoringError,
    build_model_authoring_plan,
    build_model_automation_assessment,
    build_model_automation_readiness_summary,
    compile_model_authoring_draft,
    load_model_authoring_draft,
    resolve_model_authoring_selection,
    run_prepared_mission_composition,
    scaffold_model_authoring_draft,
    write_model_authoring_draft,
)
from taoryx.model_overview import build_model_overview_catalog, render_model_overview_markdown
from taoryx.outputs import RunArtifact
from taoryx.plugins import PluginError, declared_plugin_entry_points, discover_plugins
from taoryx.scenario import ScenarioCompileError, ScenarioCompiler
from taoryx.simulation_runtime_bundle import build_simulation_runtime_bundle, write_composition_run_artifact
from taoryx.simulation_runtime_catalog import SimulationRuntimeScenario, load_simulation_runtime_catalog
from taoryx.simulation_runtime_contracts import SimulationRuntimeStatus
from taoryx.simulation_runtime_doctor import doctor_scenario
from taoryx.simulation_runtime_manifest import (
    SimulationRuntimeManifestCompatibilityError,
    artifact_inventory,
    build_simulation_runtime_run_manifest,
    default_runtime_identity,
    read_simulation_runtime_run_manifest,
    source_input_record,
)
from taoryx.table_explorer import InterpolationExplanation, TableInspection, explain_interpolation, inspect_table_file
from taoryx.trajectory import (
    FamilyCatalog,
    ResolvedCase,
    diff_resolved_cases,
    load_case_intent,
    load_family_catalog,
    resolve_case,
)
from taoryx.trajectory.configuration_contract import PreparedTrajectoryConfiguration
from taoryx.trajectory.evaluation import TrajectoryEvaluation
from taoryx.trajectory.execution_contract import MissionCompositionOutputSelection
from taoryx.trajectory.resolution import ResolutionError
from taoryx.vehicle_batch_execution import execute_vehicle_composition_batch
from taoryx.vehicle_composition import (
    CompiledVehicleComposition,
    VehicleCompositionError,
    compile_vehicle_composition,
    load_compiled_vehicle_composition,
    load_vehicle_composition_request,
    resolve_vehicle_composition_interface_contract,
)
from taoryx.vehicle_composition_registry import (
    ResolvedVehicleCompositionCatalog,
    build_vehicle_composition_topology_report,
    load_resolved_vehicle_composition_catalog,
)
from taoryx.vehicle_endpoint_spec import vehicle_endpoint_spec_list, verify_vehicle_endpoint
from taoryx.vehicle_execution_artifact import read_vehicle_execution_packet
from taoryx.vehicle_execution_bindings import VehicleExecutionBinding, resolve_vehicle_execution_binding
from taoryx.vehicle_execution_preflight import build_semantic_preflight_handler_report, preflight_vehicle_composition
from taoryx.vehicle_integration_intake import (
    ExistingFamilyIntakeRequest,
    NewTopologyIntakeRequest,
    StrategySelectionError,
    build_existing_family_intake_blueprint,
    build_new_topology_intake_scaffold,
)
from taoryx.vehicle_integration_readiness import (
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)
from taoryx.vehicle_interface import build_vehicle_interface_catalog_report, resolve_vehicle_interface_contract, validate_vehicle_interface_contract
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots

from .optimization_runtime import available_optimizers
from .runner import run_files


class _ArtifactInspectionVehicle(TypedDict):
    """Typed diagnostic projection for one artifact vehicle."""

    vehicle_id: str
    name: str
    kind: str
    dynamics: str
    sample_count: int
    time_start_s: float | None
    time_end_s: float | None
    channels: list[str]
    segments: list[str]


class _ArtifactInspectionPayload(TypedDict):
    """Typed diagnostic projection for a normalized run artifact."""

    schema: str
    artifact_schema_version: int
    problem: str
    scenario_identity: str | None
    vehicles: list[_ArtifactInspectionVehicle]
    event_count: int
    command_count: int
    termination: dict[str, object]
    claim_boundary: str


_OFFICIAL_PLUGIN_DISTRIBUTIONS: dict[str, str] = {
    "taoryx.cadac": "taoryx-cadac",
    "taoryx.a320": "taoryx-a320",
    "taoryx.daveml": "taoryx-daveml",
    "taoryx.debug-models": "taoryx-debug-models",
    "taoryx.f16": "taoryx-f16",
    "taoryx.hummingbird": "taoryx-hummingbird",
    "taoryx.nesc": "taoryx-nesc",
    "taoryx.passive-bodies": "taoryx-passive-bodies",
    "taoryx.simple-aero": "taoryx-simple-aero",
    "taoryx.x15": "taoryx-x15",
    "taoryx.hl20": "taoryx-hl20",
    "taoryx.source-table-fixed-wing": "taoryx-source-table-fixed-wing",
    "taoryx.reference-models": "taoryx-reference-models",
    "taoryx.dual-launch": "taoryx-dual-launch",
    "taoryx.reachability": "taoryx-reachability",
}

_DIRECT_MODEL_PLUGIN_IDS: tuple[str, ...] = (
    "taoryx.daveml",
    "taoryx.debug-models",
    "taoryx.a320",
    "taoryx.f16",
    "taoryx.hummingbird",
    "taoryx.nesc",
    "taoryx.passive-bodies",
    "taoryx.simple-aero",
    "taoryx.x15",
    "taoryx.hl20",
    "taoryx.source-table-fixed-wing",
    "taoryx.dual-launch",
)

_DEVELOPER_PLUGIN_IDS: tuple[str, ...] = (*_DIRECT_MODEL_PLUGIN_IDS, "taoryx.reachability")

_COMPATIBILITY_PLUGIN_IDS: tuple[str, ...] = (
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

_PLUGIN_INSTALL_PROFILES: dict[str, tuple[str, ...]] = {
    "core": (),
    "cadac": ("taoryx.cadac",),
    "models": _DIRECT_MODEL_PLUGIN_IDS,
    "developer": _DEVELOPER_PLUGIN_IDS,
    "compatibility": _COMPATIBILITY_PLUGIN_IDS,
    "full": (
        *_DIRECT_MODEL_PLUGIN_IDS,
        "taoryx.reference-models",
        "taoryx.reachability",
    ),
}


def _add_reachability_criteria_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--min-speed-m-s", type=float)
    parser.add_argument("--max-speed-m-s", type=float)
    parser.add_argument("--require-ground-contact", action="store_true")
    parser.add_argument("--target-x-m", type=float)
    parser.add_argument("--target-y-m", type=float)
    parser.add_argument("--target-z-m", type=float)
    parser.add_argument("--max-impact-radius-m", type=float)
    parser.add_argument("--min-impact-speed-m-s", type=float)
    parser.add_argument("--max-impact-speed-m-s", type=float)


def _add_model_selection_arguments(parser: argparse.ArgumentParser) -> None:
    """Add the exact provider-advertised selection axes used by model tools."""

    parser.add_argument("--fidelity", help="advertised fidelity ID; an advertised default is used when unambiguous")
    parser.add_argument("--realization", help="advertised executable realization ID")
    parser.add_argument("--mission", help="advertised mission-template ID")
    ####


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="taoryx", description="Typed TAOS parser and runtime")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run", help="execute a .prb problem with optional .tbl files")
    run.add_argument("problem", type=Path)
    run.add_argument("tables", type=Path, nargs="*")
    run.add_argument("--output-dir", type=Path, default=Path("."))
    run.add_argument("--report", type=Path)
    run.add_argument(
        "--artifact",
        type=Path,
        help="write the first normalized RunArtifact as JSON for inspection or plotting",
    )
    run.add_argument("--json", action="store_true")
    run.add_argument("--max-steps", type=int, default=100000)
    run.add_argument("--seed", type=int, help="base seed for *random sampling")
    run.add_argument("--profile", choices=tuple(profile.value for profile in GrammarProfile), default=GrammarProfile.TAOS96.value)
    run.add_argument("--sensor-spec", type=Path, help="provider-neutral sensor scenario sidecar")
    run.add_argument(
        "--integrator",
        choices=tuple(item.value for item in available_integrators()),
        help=("integration backend: euler for fast tests, rk4 for fixed-step runs, or scipy-* if installed"),
    )
    scenario = subparsers.add_parser("scenario", help="discover, validate, bundle, or compile Simulation Runtime scenarios")
    scenario_subparsers = scenario.add_subparsers(dest="scenario_command", required=True)
    scenario_list = scenario_subparsers.add_parser("list", help="list canonical Simulation Runtime scenarios")
    scenario_list.add_argument("--catalog", type=Path, default=Path("verification/simulation_runtime_scenario_catalog.yaml"))
    scenario_list.add_argument("--json", action="store_true")
    scenario_search = scenario_subparsers.add_parser("search", help="search canonical Simulation Runtime scenarios")
    scenario_search.add_argument("query")
    scenario_search.add_argument("--catalog", type=Path, default=Path("verification/simulation_runtime_scenario_catalog.yaml"))
    scenario_search.add_argument("--json", action="store_true")
    scenario_show = scenario_subparsers.add_parser("show", help="show one canonical Simulation Runtime scenario")
    scenario_show.add_argument("identifier")
    scenario_show.add_argument("--catalog", type=Path, default=Path("verification/simulation_runtime_scenario_catalog.yaml"))
    scenario_show.add_argument("--json", action="store_true")
    scenario_bundle = scenario_subparsers.add_parser("bundle", help="collect one scenario into a reproducible evidence bundle")
    scenario_bundle.add_argument("identifier")
    scenario_bundle.add_argument("--catalog", type=Path, default=Path("verification/simulation_runtime_scenario_catalog.yaml"))
    scenario_bundle.add_argument("--output-dir", type=Path, required=True)
    scenario_bundle.add_argument("--no-run", action="store_true")
    scenario_bundle.add_argument("--no-plots", action="store_true")
    scenario_bundle.add_argument("--json", action="store_true")
    scenario_manifest = scenario_subparsers.add_parser("manifest", help="validate a Simulation Runtime run manifest")
    scenario_manifest_subparsers = scenario_manifest.add_subparsers(dest="scenario_manifest_command", required=True)
    scenario_manifest_validate = scenario_manifest_subparsers.add_parser("validate", help="validate schema, required fields, and run identity")
    scenario_manifest_validate.add_argument("path", type=Path)
    scenario_manifest_validate.add_argument("--json", action="store_true")
    compile_scenario = scenario_subparsers.add_parser("compile", help="validate source and write a resolved scenario cache")
    compile_scenario.add_argument("problem", type=Path)
    compile_scenario.add_argument("tables", type=Path, nargs="*")
    compile_scenario.add_argument("--output", type=Path, required=True)
    compile_scenario.add_argument("--profile", choices=tuple(profile.value for profile in GrammarProfile), default=GrammarProfile.TAOS96.value)
    compile_scenario.add_argument("--seed", type=int)
    compile_scenario.add_argument("--integrator")
    compile_scenario.add_argument("--json", action="store_true")
    doctor = subparsers.add_parser("doctor", help="run the ordered Simulation Runtime setup diagnostic ladder")
    doctor.add_argument("identifier")
    doctor.add_argument("--catalog", type=Path, default=Path("verification/simulation_runtime_scenario_catalog.yaml"))
    doctor.add_argument("--json", action="store_true")
    artifact = subparsers.add_parser("artifact", help="inspect normalized run artifacts")
    artifact_subparsers = artifact.add_subparsers(dest="artifact_command", required=True)
    artifact_html = artifact_subparsers.add_parser("html", help="render a run artifact as standalone HTML")
    artifact_html.add_argument("path", type=Path)
    artifact_html.add_argument("--output", type=Path, required=True)
    artifact_html.add_argument("--vehicle")
    artifact_html.add_argument("--channel", action="append", default=[])
    artifact_inspect = artifact_subparsers.add_parser("inspect", help="summarize one normalized run artifact")
    artifact_inspect.add_argument("path", type=Path)
    artifact_inspect.add_argument("--json", action="store_true")
    artifact_validate = artifact_subparsers.add_parser("validate", help="validate one normalized run artifact against its schema")
    artifact_validate.add_argument("path", type=Path)
    artifact_validate.add_argument("--json", action="store_true")
    artifact_plot = artifact_subparsers.add_parser("plot", help="render static PNG plots from a run artifact")
    artifact_plot.add_argument("path", type=Path)
    artifact_plot.add_argument("--output-dir", type=Path, required=True)
    artifact_plot.add_argument("--vehicle")
    artifact_plot.add_argument("--channel", action="append", default=[])
    table = subparsers.add_parser("table", help="inspect TAOS table files")
    table_subparsers = table.add_subparsers(dest="table_command", required=True)
    inspect = table_subparsers.add_parser("inspect", help="inspect a .tbl file")
    inspect.add_argument("path", type=Path)
    inspect.add_argument("--table", dest="table_name")
    inspect.add_argument("--at", action="append", default=[], metavar="AXIS=VALUE")
    inspect.add_argument("--json", action="store_true")
    inspect.add_argument("--html", type=Path)
    optimizers = subparsers.add_parser("optimizers", help="inspect available numerical backends")
    optimizers_subparsers = optimizers.add_subparsers(dest="optimizer_command", required=True)
    optimizers_subparsers.add_parser("list", help="list installed optimization backends")
    integrators = subparsers.add_parser("integrators", help="inspect available integration backends")
    integrators_subparsers = integrators.add_subparsers(dest="integrator_command", required=True)
    integrators_subparsers.add_parser("list", help="list installed integration backends")
    plugins = subparsers.add_parser("plugins", help="inspect installed model and provider plug-ins")
    plugins_subparsers = plugins.add_subparsers(dest="plugins_command", required=True)
    plugins_list = plugins_subparsers.add_parser("list", help="list compatible plug-ins and their typed contributions")
    plugins_list.add_argument("--json", action="store_true")
    plugins_list.add_argument("--no-builtin", action="store_true")
    plugins_list.add_argument("--no-external", action="store_true")
    plugins_list.add_argument("--disable", action="append", default=[], metavar="PLUGIN_ID")
    plugins_entry_points = plugins_subparsers.add_parser(
        "entry-points",
        help="list declared plug-in entry points without importing plug-in targets",
    )
    plugins_entry_points.add_argument("--json", action="store_true")
    plugins_entry_points.add_argument("--no-builtin", action="store_true")
    plugins_entry_points.add_argument("--no-external", action="store_true")
    plugins_inspect = plugins_subparsers.add_parser("inspect", help="show one installed plug-in and its contributions")
    plugins_inspect.add_argument("plugin_id")
    plugins_inspect.add_argument("--json", action="store_true")
    plugins_inspect.add_argument("--no-builtin", action="store_true")
    plugins_inspect.add_argument("--no-external", action="store_true")
    plugins_inspect.add_argument("--disable", action="append", default=[], metavar="PLUGIN_ID")
    plugins_check = plugins_subparsers.add_parser(
        "check",
        help="verify an official installed distribution profile without source fallbacks",
    )
    plugins_check.add_argument(
        "--profile",
        choices=tuple(_PLUGIN_INSTALL_PROFILES),
        default="developer",
        help="installed package profile (developer excludes the compatibility aggregate; full retains it)",
    )
    plugins_check.add_argument("--json", action="store_true")
    model = subparsers.add_parser(
        "model",
        help="plan, scaffold, validate, and tune plug-in models through common contracts",
    )
    model_subparsers = model.add_subparsers(dest="model_command", required=True)
    model_list = model_subparsers.add_parser("list", help="list every composer model and registered tuning campaign")
    model_list.add_argument("--provider", help="restrict the inventory to one exact provider ID")
    model_list.add_argument("--output", type=Path)
    model_overview = model_subparsers.add_parser(
        "overview",
        help="render evidence-bounded model cards with fidelity, provenance, tuning, missions, and parameters",
    )
    model_overview.add_argument("--provider", help="restrict cards to one exact provider ID")
    model_overview.add_argument("--model", help="restrict cards to one exact model ID")
    model_overview.add_argument("--format", choices=("markdown", "json"), default="markdown")
    model_overview.add_argument("--output", type=Path)
    model_assess = model_subparsers.add_parser(
        "assess",
        help="report all-model advertisement, control, adapter, and tuning readiness",
    )
    model_assess.add_argument("--provider", help="restrict the readiness matrix to one exact provider ID")
    model_assess.add_argument("--output", type=Path)
    model_assess.add_argument(
        "--summary",
        action="store_true",
        help="emit a concise per-realization control and tuning readiness inventory",
    )
    model_plan = model_subparsers.add_parser(
        "plan",
        help="join model data, controls, navigation, segments, adapters, and tuning readiness",
    )
    model_plan.add_argument("provider_id")
    model_plan.add_argument("model_id")
    _add_model_selection_arguments(model_plan)
    model_plan.add_argument("--output", type=Path)
    model_scaffold = model_subparsers.add_parser(
        "scaffold",
        help="generate an editable plain-value YAML or JSON mission draft",
    )
    model_scaffold.add_argument("provider_id")
    model_scaffold.add_argument("model_id")
    _add_model_selection_arguments(model_scaffold)
    model_scaffold.add_argument("--draft-id")
    model_scaffold.add_argument("--configuration-id")
    model_scaffold.add_argument("--output", type=Path, required=True)
    model_compile = model_subparsers.add_parser(
        "compile",
        help="compile a plain-value draft through the exact provider schema",
    )
    model_compile.add_argument("draft", type=Path)
    model_compile.add_argument("--output", type=Path)
    model_run = model_subparsers.add_parser(
        "run",
        help="run a prepared configuration through its provider-owned common batch runner",
    )
    model_run.add_argument("provider_id", help="exact provider that validated the prepared configuration")
    model_run.add_argument("prepared", type=Path, help="JSON written by 'taoryx model compile'")
    model_run.add_argument("--request-id", help="caller correlation ID; defaults to the configuration ID")
    model_run.add_argument("--output-mode", choices=("core", "selected", "all"), default="core")
    model_run.add_argument("--channel", action="append", default=[], help="request one telemetry channel (requires --output-mode selected)")
    model_run.add_argument("--telemetry-group", action="append", default=[], help="request one telemetry group (requires --output-mode selected)")
    model_run.add_argument("--cadence-s", type=float, help="requested output cadence in seconds")
    model_run.add_argument("--maximum-samples-per-object", type=int, help="cap samples returned for each object")
    model_run.add_argument("--maximum-objects", type=int, help="cap returned primary and spawned objects")
    model_run.add_argument("--no-events", action="store_true", help="omit event records when the provider supports it")
    model_run.add_argument("--no-segments", action="store_true", help="omit segment spans when the provider supports it")
    model_run.add_argument("--no-spawned-objects", action="store_true", help="return only the primary object")
    model_run.add_argument("--output", type=Path, help="write the discriminated trajectory/failure response as JSON")
    model_tune = model_subparsers.add_parser(
        "tune",
        help="run a plug-in campaign through the common stop-at-first-blocker tuning pipeline",
    )
    model_tune.add_argument("provider_id")
    model_tune.add_argument("model_id")
    _add_model_selection_arguments(model_tune)
    model_tune.add_argument("--campaign", help="registered campaign ID; inferred only when exactly one applies")
    model_tune.add_argument(
        "--cache-dir",
        type=Path,
        default=Path("build/controller-cache"),
        help="content-addressed tuning cache (default: build/controller-cache)",
    )
    model_tune.add_argument("--no-cache", action="store_true", help="rerun the campaign without reading or writing cache")
    model_tune.add_argument("--output", type=Path)
    model_subparsers.add_parser(
        "endpoint-specs",
        help="list focused vertical proofs for registered non-physical trajectory workflows",
    ).add_argument("--output", type=Path)
    model_verify = model_subparsers.add_parser(
        "verify",
        help="verify one checked-in trajectory-workflow draft through provider validation and its common runner",
    )
    model_verify.add_argument("endpoint_id", help="ID from 'taoryx model endpoint-specs'")
    model_verify.add_argument(
        "--execute",
        action="store_true",
        help="run the exact workflow through its advertised common batch runner after static verification",
    )
    model_verify.add_argument("--output", type=Path)
    reachability = subparsers.add_parser("reachability", help="inspect Alpha 3 reachability catalogs")
    reachability_subparsers = reachability.add_subparsers(dest="reachability_command", required=True)
    reachability_list = reachability_subparsers.add_parser("list", help="list reachability catalog entries")
    reachability_list.add_argument("kind", choices=("profiles", "families", "semantics"))
    reachability_list.add_argument("--catalog", type=Path)
    reachability_inspect = reachability_subparsers.add_parser("inspect", help="inspect one reachability catalog entry")
    reachability_inspect.add_argument("kind", choices=("profile", "family", "semantic"))
    reachability_inspect.add_argument("identifier")
    reachability_inspect.add_argument("--catalog", type=Path)
    reachability_run = reachability_subparsers.add_parser("run", help="run the reduced-order rocket/glide envelope fixture")
    reachability_run.add_argument(
        "--fidelity",
        choices=("point_mass_3dof", "pseudo_6dof", "rigid_body_6dof", "rigid_body_6dof_surface_allocated"),
        default="point_mass_3dof",
    )
    reachability_run.add_argument("--workers", type=int, default=1)
    reachability_run.add_argument("--azimuth-deg", type=float, action="append", default=[])
    reachability_run.add_argument("--elevation-deg", type=float, action="append", default=[])
    reachability_run.add_argument("--bank-deg", type=float, action="append", default=[])
    reachability_run.add_argument("--step-size-s", type=float, default=0.25)
    reachability_run.add_argument("--horizon-s", type=float, default=120.0)
    reachability_run.add_argument("--output", type=Path)
    reachability_run.add_argument("--include-trajectories", action="store_true")
    reachability_run.add_argument("--omit-trajectories", action="store_true")
    _add_reachability_criteria_arguments(reachability_run)
    reachability_timeout = reachability_subparsers.add_parser("rerun-timeouts", help="rerun only candidates that reached the prior horizon")
    reachability_timeout.add_argument("envelope", type=Path)
    reachability_timeout.add_argument("--horizon-s", type=float, required=True)
    reachability_timeout.add_argument("--step-size-s", type=float)
    reachability_timeout.add_argument("--workers", type=int)
    reachability_timeout.add_argument("--output", type=Path, required=True)
    reachability_plot = reachability_subparsers.add_parser("plot", help="render Matplotlib plots from an envelope artifact")
    reachability_plot.add_argument("path", type=Path)
    reachability_plot.add_argument("--output-dir", type=Path, required=True)
    reachability_plot.add_argument("--compare", type=Path, action="append", default=[])
    reachability_plot.add_argument("--dpi", type=int, default=140)
    reachability_x15 = reachability_subparsers.add_parser("x15", help="run the X-15-scaled reachability tier demonstration")
    reachability_x15.add_argument("--output-dir", type=Path, required=True)
    reachability_x15.add_argument("--workers", type=int, default=1)
    reachability_x15.add_argument("--step-size-s", type=float, default=0.5)
    reachability_x15.add_argument("--horizon-s", type=float, default=120.0)
    reachability_x15.add_argument("--dpi", type=int, default=140)
    _add_reachability_criteria_arguments(reachability_x15)
    reachability_x15_native = reachability_subparsers.add_parser(
        "x15-native-replay", help="replay selected X-15 envelope points through native rigid-body cases"
    )
    reachability_x15_native.add_argument("envelope", type=Path)
    reachability_x15_native.add_argument("--output-dir", type=Path, required=True)
    reachability_x15_native.add_argument("--max-points", type=int, default=4)
    reachability_x15_native.add_argument("--duration-s", type=float, default=0.01)
    reachability_x15_native.add_argument("--max-steps", type=int, default=50)
    catalog = subparsers.add_parser("catalog", help="inspect Alpha 2 vehicle-family catalogs")
    catalog_subparsers = catalog.add_subparsers(dest="catalog_command", required=True)
    catalog_list = catalog_subparsers.add_parser("list", help="list catalog entries")
    catalog_list.add_argument("kind", choices=("families",))
    catalog_list.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    family = subparsers.add_parser("family", help="inspect one Alpha 2 vehicle family")
    family_subparsers = family.add_subparsers(dest="family_command", required=True)
    family_inspect = family_subparsers.add_parser("inspect", help="show family metadata")
    family_inspect.add_argument("family_id")
    family_inspect.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    family_schema = family_subparsers.add_parser("schema", help="export family schemas")
    family_schema.add_argument("family_id")
    family_schema.add_argument("--exposure", choices=("common",), default="common")
    family_schema.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    vehicle = subparsers.add_parser("vehicle", help="inspect the unified vehicle-to-trajectory composition registry")
    vehicle_subparsers = vehicle.add_subparsers(dest="vehicle_command", required=True)
    vehicle_catalog = vehicle_subparsers.add_parser(
        "catalog",
        help="export the versioned discovery document for all registered vehicles",
    )
    vehicle_catalog.add_argument(
        "--detail",
        choices=("summary", "full"),
        default="summary",
        help="summary lists discoverable identities; full embeds every composition descriptor",
    )
    vehicle_subparsers.add_parser("list", help="list registered vehicles, families, tiers, and mission templates")
    vehicle_release_catalog = vehicle_subparsers.add_parser(
        "release-catalog",
        help="build a hash-bound inventory from existing valid result packets without rerunning them",
    )
    vehicle_release_catalog.add_argument("directory", type=Path, help="root directory containing normalized result packets")
    vehicle_release_catalog.add_argument("--output", type=Path, required=True, help="destination release-catalog JSON path")
    vehicle_subparsers.add_parser(
        "interface-report",
        help="validate every registered vehicle/fidelity interface against its execution bindings",
    )
    vehicle_subparsers.add_parser(
        "topology-report",
        help="audit public parameter, interface, observation, and objective value-space topology",
    )
    vehicle_subparsers.add_parser(
        "semantic-preflight-handler-report",
        help="audit catalog-declared semantic translators against installed preflight handlers",
    )
    vehicle_maturity_report = vehicle_subparsers.add_parser(
        "maturity-report",
        help="join Mission Composition topology, authoring, execution, and parity coverage without evidence promotion",
    )
    vehicle_maturity_report.add_argument(
        "--check-execution-witnesses",
        action="store_true",
        help="compile every checked-in endpoint witness and open declared episodes before reporting",
    )
    vehicle_maturity_report.add_argument(
        "--execute-batch-witnesses",
        action="store_true",
        help=("also run every checked-in batch witness and verify its declared action-trace disposition; implies --check-execution-witnesses"),
    )
    vehicle_maturity_report.add_argument(
        "--execute-parity-witnesses",
        action="store_true",
        help="replay one action trace through every exact pair with registered batch/episode parity evidence",
    )
    vehicle_maturity_report.add_argument(
        "--results-dir",
        type=Path,
        help="optionally index existing normalized result artifacts as part of the maturity report",
    )
    vehicle_maturity_report.add_argument(
        "--retain-batch-results-dir",
        type=Path,
        help="retain all executed batch-witness packets in an empty directory and index the generated corpus",
    )
    vehicle_witness_report = vehicle_subparsers.add_parser(
        "witness-report",
        help="compile or run exact public endpoint witnesses, optionally scoped to one family or endpoint",
    )
    vehicle_witness_report.add_argument(
        "--execute-batch",
        action="store_true",
        help="run selected batch witnesses through their public compose-to-run entry point",
    )
    vehicle_witness_report.add_argument(
        "--results-dir",
        type=Path,
        help="retain generated batch witness packets in an empty directory and write an aggregate release catalog",
    )
    witness_selection = vehicle_witness_report.add_mutually_exclusive_group()
    witness_selection.add_argument(
        "--family",
        action="append",
        metavar="FAMILY_ID",
        help="limit to one family; repeat to select several families",
    )
    witness_selection.add_argument(
        "--witness",
        action="append",
        metavar="WITNESS_ID",
        help="limit to one exact checked-in endpoint witness; repeat to select several witnesses",
    )
    vehicle_subparsers.add_parser(
        "endpoint-specs",
        help="list focused vertical contracts that join composition, runtime, advertisements, outputs, and tuning",
    )
    vehicle_verify = vehicle_subparsers.add_parser(
        "verify",
        help="verify one focused vehicle endpoint across its declared composition, runtime, advertisements, outputs, and tuner",
    )
    vehicle_verify.add_argument("endpoint_id", help="ID from 'taoryx vehicle endpoint-specs'")
    vehicle_verify.add_argument(
        "--execute",
        action="store_true",
        help="run the endpoint's exact batch factory or initialize its exact interactive episode after static verification",
    )
    vehicle_verify.add_argument(
        "--tune",
        action="store_true",
        help="run the endpoint's declared controller campaign after verifying its advertised tuning operations",
    )
    vehicle_verify_cache = vehicle_verify.add_mutually_exclusive_group()
    vehicle_verify_cache.add_argument(
        "--cache-dir",
        type=Path,
        default=Path("build/controller-tuning-cache"),
        help="content-addressed controller-campaign cache directory used only with --tune",
    )
    vehicle_verify_cache.add_argument(
        "--no-cache",
        action="store_true",
        help="with --tune, run the campaign without creating or reading a cache artifact",
    )
    vehicle_verify.add_argument(
        "--results-dir",
        type=Path,
        help=("retain the exact executed batch packet in an empty directory, including controller/tuning provenance; requires --execute"),
    )
    vehicle_verify.add_argument("--output", type=Path, help="optional JSON destination for the focused report")
    vehicle_inspect = vehicle_subparsers.add_parser("inspect", help="show one vehicle's composition contract")
    vehicle_inspect.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_describe = vehicle_subparsers.add_parser(
        "describe",
        help="export one vehicle's complete composition, interface, and execution descriptor",
    )
    vehicle_describe.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_missions = vehicle_subparsers.add_parser("missions", help="list one vehicle's declared mission templates")
    vehicle_missions.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_segment = vehicle_subparsers.add_parser("segment", help="show one reusable segment contract")
    vehicle_segment.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_segment.add_argument("segment_id", help="declared segment contract ID")
    vehicle_parameters = vehicle_subparsers.add_parser("parameters", help="query public initialization or segment parameters")
    vehicle_parameters.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_parameters.add_argument(
        "--scope",
        choices=("episode_reset", "segment", "variant_configuration"),
        help="limit the query to one public parameter scope",
    )
    vehicle_authoring = vehicle_subparsers.add_parser(
        "authoring",
        help="show the Mission Composition authoring worklist for one vehicle family",
    )
    vehicle_authoring.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_authoring_template = vehicle_subparsers.add_parser(
        "authoring-template",
        help="export a no-invented-default composition authoring kit for one mission and fidelity",
    )
    vehicle_authoring_template.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_authoring_template.add_argument("mission_id", help="declared mission template ID")
    vehicle_authoring_template.add_argument(
        "fidelity",
        choices=(
            "point_mass_3dof",
            "pseudo_6dof",
            "rigid_body_6dof_direct_wrench",
            "rigid_body_6dof_surface_allocated",
        ),
    )
    vehicle_mission = vehicle_subparsers.add_parser(
        "mission",
        help="inspect, author, or validate one reusable mission through the composition compiler",
    )
    vehicle_mission_subparsers = vehicle_mission.add_subparsers(dest="vehicle_mission_command", required=True)
    vehicle_mission_inspect = vehicle_mission_subparsers.add_parser(
        "inspect",
        help="show one mission's selectable initialization, segment, graph, and fidelity contracts",
    )
    vehicle_mission_inspect.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_mission_inspect.add_argument("mission_id", help="declared mission template ID")
    vehicle_mission_create = vehicle_mission_subparsers.add_parser(
        "create",
        help="export a no-invented-default composition authoring kit for one mission and fidelity",
    )
    vehicle_mission_create.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_mission_create.add_argument("mission_id", help="declared mission template ID")
    vehicle_mission_create.add_argument(
        "fidelity",
        choices=(
            "point_mass_3dof",
            "pseudo_6dof",
            "rigid_body_6dof_direct_wrench",
            "rigid_body_6dof_surface_allocated",
        ),
    )
    vehicle_mission_create.add_argument("--output", type=Path, help="optional JSON path for the authoring kit")
    vehicle_mission_validate = vehicle_mission_subparsers.add_parser(
        "validate",
        help="compile and inspect a mission composition request without running a vehicle",
    )
    vehicle_mission_validate.add_argument("request", type=Path, help="YAML or JSON vehicle composition request")
    vehicle_mission_validate.add_argument(
        "--compiled-output",
        type=Path,
        help="optional JSON path for the fingerprinted compiled composition",
    )
    vehicle_variant = vehicle_subparsers.add_parser(
        "variant",
        help="validate or resolve a bounded runtime-backed vehicle variant",
    )
    vehicle_variant_subparsers = vehicle_variant.add_subparsers(dest="vehicle_variant_command", required=True)
    vehicle_variant_validate = vehicle_variant_subparsers.add_parser(
        "validate",
        help="validate a request's bounded variant and report its runtime provenance without running it",
    )
    vehicle_variant_validate.add_argument("request", type=Path, help="YAML or JSON vehicle composition request")
    vehicle_variant_resolve = vehicle_variant_subparsers.add_parser(
        "resolve",
        help="resolve a request into an immutable compiled composition containing its variant manifest",
    )
    vehicle_variant_resolve.add_argument("request", type=Path, help="YAML or JSON vehicle composition request")
    vehicle_variant_resolve.add_argument("--output", type=Path, required=True, help="compiled composition JSON output")
    vehicle_subparsers.add_parser(
        "authoring-all",
        help="aggregate Mission Composition authoring worklists without collapsing family-specific gaps",
    )
    vehicle_intake = vehicle_subparsers.add_parser(
        "intake",
        help="generate a non-promotable four-tier intake package for a new vehicle or topology",
    )
    vehicle_intake_subparsers = vehicle_intake.add_subparsers(dest="vehicle_intake_mode", required=True)
    vehicle_intake_existing = vehicle_intake_subparsers.add_parser(
        "existing-family",
        help="select an explicit existing-family strategy and export its data/adapter work package",
    )
    vehicle_intake_existing.add_argument("--family-id", required=True)
    vehicle_intake_existing.add_argument("--physical-family", required=True)
    vehicle_intake_existing.add_argument("--mission-overlay", required=True)
    vehicle_intake_existing.add_argument("--adapter-id", required=True)
    vehicle_intake_existing.add_argument("--strategy-id")
    vehicle_intake_existing.add_argument("--output", type=Path)
    vehicle_intake_new = vehicle_intake_subparsers.add_parser(
        "new-topology",
        help="export a strategy-definition scaffold without inventing a new vehicle model",
    )
    vehicle_intake_new.add_argument("--family-id", required=True)
    vehicle_intake_new.add_argument("--physical-family", required=True)
    vehicle_intake_new.add_argument("--strategy-id", required=True)
    vehicle_intake_new.add_argument("--topology-summary", required=True)
    vehicle_intake_new.add_argument("--output", type=Path)
    vehicle_integration = vehicle_subparsers.add_parser(
        "integration",
        help="report source-family onboarding readiness and staged promotion evidence",
    )
    vehicle_integration_subparsers = vehicle_integration.add_subparsers(dest="vehicle_integration_command", required=True)
    vehicle_integration_readiness = vehicle_integration_subparsers.add_parser(
        "readiness",
        help="check source-manifest and four-tier metadata readiness without executing a plant",
    )
    vehicle_integration_readiness.add_argument("family", help="supported source family ID or 'all'")
    vehicle_integration_readiness.add_argument("--output", type=Path, help="optional JSON report destination")
    vehicle_integration_pipeline = vehicle_integration_subparsers.add_parser(
        "pipeline",
        help="report staged source-family integration evidence and explicit promotion gates",
    )
    vehicle_integration_pipeline.add_argument("family", help="supported source family ID or 'all'")
    vehicle_integration_pipeline.add_argument("--output", type=Path, help="optional JSON report destination")
    vehicle_integration_pipeline.add_argument(
        "--packet-dir",
        type=Path,
        help="optionally write one reproducible source-integration packet per selected family",
    )
    vehicle_integration_pipeline.add_argument(
        "--exclude-source-package",
        action="store_true",
        help="when writing packets, retain source hashes but do not copy pinned source packages",
    )
    vehicle_integration_pipeline.add_argument(
        "--allow-blocked",
        action="store_true",
        help="return success after reporting blockers; never changes any evidence or promotion status",
    )
    vehicle_schema = vehicle_subparsers.add_parser("schema", help="export one vehicle composition schema section")
    vehicle_schema.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_schema.add_argument("kind", choices=("initialization", "segments", "missions"))
    vehicle_endpoints = vehicle_subparsers.add_parser(
        "endpoints",
        help="list the exact batch and interactive factories declared for one vehicle family",
    )
    vehicle_endpoints.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_interface = vehicle_subparsers.add_parser(
        "interface",
        help="resolve the versioned parameter, action, status, resource, and observation contract for one fidelity",
    )
    vehicle_interface.add_argument("identifier", help="family ID or native vehicle registry ID")
    vehicle_interface.add_argument(
        "fidelity",
        choices=(
            "point_mass_3dof",
            "pseudo_6dof",
            "rigid_body_6dof_direct_wrench",
            "rigid_body_6dof_surface_allocated",
        ),
    )
    vehicle_composition_interface = vehicle_subparsers.add_parser(
        "interface-composition",
        help="resolve the exact fingerprinted interface selected by a compiled composition",
    )
    vehicle_composition_interface.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_compose = vehicle_subparsers.add_parser(
        "compose",
        help="validate a vehicle initialization and ordered mission into a semantic adapter handoff",
    )
    vehicle_compose.add_argument("request", type=Path, help="YAML or JSON vehicle composition request")
    vehicle_compose.add_argument("--output", type=Path, help="write the compiled semantic scenario as JSON")
    vehicle_validate_examples = vehicle_subparsers.add_parser(
        "validate-examples",
        help="compile and inspect every checked-in composition witness without running a vehicle",
    )
    vehicle_validate_examples.add_argument(
        "--directory",
        type=Path,
        default=Path("examples/vehicle_composition"),
        help="directory containing YAML or JSON composition requests",
    )
    vehicle_validate_examples.add_argument(
        "--require-preflight",
        action="store_true",
        help="return failure unless every witness is translation-ready",
    )
    vehicle_lower = vehicle_subparsers.add_parser(
        "lower",
        help="bind a compiled semantic scenario to its declared native adapter without fallback",
    )
    vehicle_lower.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_results = vehicle_subparsers.add_parser(
        "results",
        help="index normalized evaluation artifacts below one output directory",
    )
    vehicle_results.add_argument("directory", type=Path, help="directory to scan recursively for evaluation.json")
    vehicle_preflight = vehicle_subparsers.add_parser(
        "preflight",
        help="fail closed when a semantic composition cannot map to its declared native mission translator",
    )
    vehicle_preflight.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_materialize = vehicle_subparsers.add_parser(
        "materialize",
        help="write exact language-backed native inputs for a translation-ready composition",
    )
    vehicle_materialize.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_materialize.add_argument("--output-dir", type=Path, required=True, help="directory for disposable native inputs")
    vehicle_run = vehicle_subparsers.add_parser(
        "run",
        help="execute a translation-ready composition through its exact declared source-owned batch binding",
    )
    vehicle_run.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_run.add_argument("--output-dir", type=Path, required=True, help="directory for immutable nominal-run evidence")
    vehicle_run.add_argument("--max-steps", type=int, help="optional positive runtime step limit")
    vehicle_replay_policy = vehicle_subparsers.add_parser(
        "replay-policy",
        help="replay a persisted semantic action trace through the exact selected composition episode",
    )
    vehicle_replay_policy.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_replay_policy.add_argument("trace", type=Path, help="persisted composition policy trace JSON")
    vehicle_replay_policy.add_argument("--output", type=Path, help="optional replay verdict JSON path")
    vehicle_batch_episode_parity = vehicle_subparsers.add_parser(
        "batch-episode-parity",
        help="compare one persisted semantic action trace through its exact declared batch and episode paths",
    )
    vehicle_batch_episode_parity.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_batch_episode_parity.add_argument("trace", type=Path, help="persisted composition policy trace JSON")
    vehicle_batch_episode_parity.add_argument("--output", type=Path, help="optional parity verdict JSON path")
    vehicle_episode_info = vehicle_subparsers.add_parser(
        "episode-info",
        help="open one declared episode factory and export its initial committed-truth frame",
    )
    vehicle_episode_info.add_argument("composition", type=Path, help="compiled vehicle composition JSON")
    vehicle_episode_info.add_argument("--seed", type=int, help="optional declared episode seed")
    vehicle_result = vehicle_subparsers.add_parser(
        "result",
        help="read and validate one normalized Mission Composition evaluation artifact",
    )
    vehicle_result.add_argument("output_dir", type=Path, help="composition-run artifact directory")
    vehicle_result.add_argument(
        "--composition",
        type=Path,
        help="optional compiled composition to fingerprint-bind to the evaluation",
    )
    case = subparsers.add_parser("case", help="resolve and inspect Alpha 2 case intents")
    case_subparsers = case.add_subparsers(dest="case_command", required=True)
    case_validate = case_subparsers.add_parser("validate", help="validate a case intent")
    case_validate.add_argument("path", type=Path)
    case_validate.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    case_resolve = case_subparsers.add_parser("resolve", help="resolve a case intent")
    case_resolve.add_argument("path", type=Path)
    case_resolve.add_argument("--output", type=Path, required=True)
    case_resolve.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    case_explain = case_subparsers.add_parser("explain", help="explain resolved parameter provenance")
    case_explain.add_argument("path", type=Path)
    case_explain.add_argument("--parameter")
    case_explain.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    case_diff = case_subparsers.add_parser("diff", help="diff two case intents")
    case_diff.add_argument("left", type=Path)
    case_diff.add_argument("right", type=Path)
    case_diff.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    schema = subparsers.add_parser("schema", help="export Alpha 2 case schemas")
    schema_subparsers = schema.add_subparsers(dest="schema_command", required=True)
    schema_export = schema_subparsers.add_parser("export", help="export parameters, controls, or observations")
    schema_export.add_argument("path", type=Path)
    schema_export.add_argument("--kind", choices=("parameters", "controls", "observations"), required=True)
    schema_export.add_argument("--catalog", type=Path, default=Path("verification/alpha2_family_catalog.yaml"))
    schema_export.add_argument("--output", type=Path)
    daveml = subparsers.add_parser("daveml", help="run promoted DAVE-ML family smoke paths")
    daveml_subparsers = daveml.add_subparsers(dest="daveml_command", required=True)
    daveml_smoke = daveml_subparsers.add_parser("smoke", help="verify a DAVE-ML family smoke evidence chain")
    daveml_smoke.add_argument(
        "--family",
        choices=("a320_openap_3dof", "reference_f16_s119", "reference_hl20_mod_k", "reference_nesc_two_stage_rocket"),
        required=True,
    )
    daveml_smoke.add_argument("--output", type=Path)
    daveml_composite = daveml_subparsers.add_parser("composite-smoke", help="verify a bounded surrogate-composite smoke path")
    daveml_composite.add_argument("--family", choices=("a320_openap_jsbsim_pseudo6dof",), required=True)
    daveml_composite.add_argument("--output", type=Path)
    arguments = parser.parse_args(argv)
    if arguments.command == "scenario":
        return _scenario_command(arguments)
    if arguments.command == "doctor":
        return _doctor_command(arguments)
    if arguments.command == "artifact":
        return _render_artifact(arguments)
    if arguments.command == "table":
        return _inspect_table(arguments)
    if arguments.command == "optimizers":
        for backend in available_optimizers():
            print(backend.value)
        return 0
    if arguments.command == "integrators":
        for integrator_name, description in available_integrator_descriptions():
            print(f"{integrator_name.value}\t{description}")
        return 0
    if arguments.command == "plugins":
        return _plugins_command(arguments)
    if arguments.command == "model":
        return _model_command(arguments)
    if arguments.command == "reachability":
        return _reachability_command(arguments)
    if arguments.command == "catalog":
        return _catalog_command(arguments)
    if arguments.command == "family":
        return _family_command(arguments)
    if arguments.command == "vehicle":
        return _vehicle_command(arguments)
    if arguments.command == "case":
        return _case_command(arguments)
    if arguments.command == "schema":
        return _schema_command(arguments)
    if arguments.command == "daveml":
        return _daveml_command(arguments)
    if arguments.max_steps <= 0:
        parser.error("--max-steps must be positive")
    report = run_files(
        arguments.problem,
        tuple(arguments.tables),
        output_dir=arguments.output_dir,
        max_steps=arguments.max_steps,
        integrator=arguments.integrator,
        seed=arguments.seed,
        profile=arguments.profile,
        sensor_spec=arguments.sensor_spec,
    )
    normalized_artifact: Path | None = None
    if arguments.artifact:
        if not report.artifacts:
            print("error: artifact-write-failed: runtime did not produce a normalized RunArtifact")
            return report.exit_code if report.exit_code != 0 else 2
        try:
            normalized_artifact = report.artifacts[0].write_json(arguments.artifact)
        except (OSError, TypeError, ValueError) as error:
            print(f"error: artifact-write-failed: {error}")
            return 2
        if not arguments.json:
            print(f"artifact: {normalized_artifact}")
    report_payload = report.as_dict()
    if normalized_artifact is not None:
        report_payload["normalized_artifact"] = str(normalized_artifact)
    if arguments.report:
        try:
            arguments.report.parent.mkdir(parents=True, exist_ok=True)
            arguments.report.write_text(json.dumps(report_payload, indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            print(f"error: report-write-failed: {error}")
            return 2
    try:
        run_manifest = _write_source_run_manifest(arguments, report, normalized_artifact)
        report_payload["run_manifest"] = str(run_manifest)
    except (OSError, TypeError, ValueError) as error:
        print(f"error: run-manifest-write-failed: {error}")
        return 2
    if arguments.json:
        print(json.dumps(report_payload, indent=2))
    else:
        for diagnostic in report.diagnostics:
            location = diagnostic.location
            prefix = f"{location.path}:{location.line}: " if location else ""
            print(f"{diagnostic.severity}: {prefix}{diagnostic.code}: {diagnostic.message}")
        print(f"executed {report.cases} case(s), exit={report.exit_code}")
        for output in report.outputs:
            print(f"output: {output}")
    return report.exit_code


def _plugins_command(arguments: argparse.Namespace) -> int:
    """Render a non-executing scan of installed Taoryx plug-ins."""

    if arguments.plugins_command == "check":
        return _plugin_installation_check_command(arguments)
    if arguments.plugins_command == "entry-points":
        try:
            declarations = declared_plugin_entry_points(
                include_builtin=not arguments.no_builtin,
                include_external=not arguments.no_external,
            )
        except PluginError as error:
            print(f"error: plugin-entry-point-discovery-failed: {error}")
            return 2
        entry_point_payload: dict[str, object] = {
            "schema": "taoryx.plugin-entry-point-catalog/v1",
            "entry_points": [item.public_dict() for item in declarations],
        }
        if arguments.json:
            print(json.dumps(entry_point_payload, indent=2, sort_keys=True))
            return 0
        raw_entry_points = entry_point_payload["entry_points"]
        assert isinstance(raw_entry_points, list)
        for item in raw_entry_points:
            assert isinstance(item, Mapping)
            distribution = item["distribution"] or "unknown-distribution"
            version = item["version"] or "unknown-version"
            print(f"{item['plugin_id']}\t{item['target']}\t{distribution}=={version}\t{item['origin']}")
        return 0
    try:
        catalog = discover_plugins(
            include_builtin=not arguments.no_builtin,
            include_external=not arguments.no_external,
            disabled=tuple(arguments.disable),
            strict=False,
        )
        payload: dict[str, object] = catalog.public_dict()
        if arguments.plugins_command == "inspect":
            plugin_metadata = catalog.plugin(arguments.plugin_id)
            payload = {
                "schema": "taoryx.plugin-inspection/v1",
                "api_version": payload["api_version"],
                "catalog_fingerprint": catalog.fingerprint,
                "plugin": plugin_metadata.public_dict(),
                "plugin_revision": catalog.plugin_revision(plugin_metadata.id).public_dict(),
                "contributions": [
                    item.public_dict()
                    for item in catalog.contributions
                    if item.plugin.id == plugin_metadata.id
                ],
                "diagnostics": [
                    item.public_dict()
                    for item in catalog.diagnostics
                    if item.plugin_id == plugin_metadata.id
                ],
            }
    except (KeyError, PluginError, TypeError, ValueError) as error:
        print(f"error: plugin-discovery-failed: {error}")
        return 2
    if arguments.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    raw_plugins = payload.get("plugins") if arguments.plugins_command == "list" else [payload.get("plugin")]
    plugin_items = raw_plugins if isinstance(raw_plugins, list) else []
    raw_contributions = payload.get("contributions")
    contributions = raw_contributions if isinstance(raw_contributions, list) else []
    raw_diagnostics = payload.get("diagnostics")
    diagnostics = raw_diagnostics if isinstance(raw_diagnostics, list) else []
    raw_revisions = payload.get("plugin_revisions") if arguments.plugins_command == "list" else [payload.get("plugin_revision")]
    revisions = raw_revisions if isinstance(raw_revisions, list) else []
    revision_by_plugin = {item["plugin_id"]: item for item in revisions if isinstance(item, Mapping) and isinstance(item.get("plugin_id"), str)}
    for plugin_item in plugin_items:
        if not isinstance(plugin_item, Mapping):
            continue
        revision = revision_by_plugin.get(plugin_item["id"])
        fingerprint = revision.get("fingerprint") if isinstance(revision, Mapping) else None
        suffix = f"\tREV {fingerprint[:12]}" if isinstance(fingerprint, str) else ""
        print(
            f"{plugin_item['id']}\t{plugin_item['package']}=={plugin_item['version']}"
            f"\tAPI {plugin_item['api_version']}{suffix}"
        )
        for contribution in contributions:
            if isinstance(contribution, Mapping) and contribution.get("plugin_id") == plugin_item["id"]:
                print(f"  {contribution['kind']}\t{contribution['id']}")
    for diagnostic in diagnostics:
        if isinstance(diagnostic, Mapping) and diagnostic.get("status") not in {"loaded", "disabled"}:
            print(f"{diagnostic['status']}: {diagnostic['plugin_id']}: {diagnostic['message']}")
    return 0
    ####


def _plugin_installation_check_command(arguments: argparse.Namespace) -> int:
    """Verify installed distributions, entry-point declarations, and registrations."""

    expected_plugin_ids = _PLUGIN_INSTALL_PROFILES[arguments.profile]
    expected_distributions = (
        "taoryx",
        *(_OFFICIAL_PLUGIN_DISTRIBUTIONS[plugin_id] for plugin_id in expected_plugin_ids),
    )
    distribution_records: list[dict[str, object]] = []
    distribution_versions: dict[str, str | None] = {}
    for distribution in expected_distributions:
        try:
            version = importlib_metadata.version(distribution)
        except importlib_metadata.PackageNotFoundError:
            version = None
        distribution_versions[distribution] = version
        distribution_records.append(
            {
                "distribution": distribution,
                "installed": version is not None,
                "version": version,
            }
        )

    entry_point_discovery_errors: list[str] = []
    if expected_plugin_ids:
        try:
            declared_entry_points = {
                item.plugin_id: item
                for item in declared_plugin_entry_points(include_builtin=False)
            }
        except PluginError as error:
            declared_entry_points = {}
            entry_point_discovery_errors.append(str(error))
        catalog = discover_plugins(include_builtin=False, strict=False)
        loaded_plugins = {plugin.id: plugin for plugin in catalog.plugins}
        contributions = catalog.contributions
        failed_diagnostics = [
            diagnostic.public_dict()
            for diagnostic in catalog.diagnostics
            if diagnostic.plugin_id in expected_plugin_ids and diagnostic.status not in {"loaded", "disabled"}
        ]
    else:
        declared_entry_points = {}
        loaded_plugins = {}
        contributions = ()
        failed_diagnostics = []

    plugin_records: list[dict[str, object]] = []
    for plugin_id in expected_plugin_ids:
        distribution = _OFFICIAL_PLUGIN_DISTRIBUTIONS[plugin_id]
        declaration = declared_entry_points.get(plugin_id)
        plugin = loaded_plugins.get(plugin_id)
        loaded = plugin is not None
        entry_point_declared = declaration is not None
        entry_point_distribution_matches = (
            declaration is not None
            and declaration.distribution is not None
            and _normalized_distribution_name(declaration.distribution) == _normalized_distribution_name(distribution)
        )
        entry_point_version_matches = declaration is not None and declaration.version == distribution_versions[distribution]
        package_matches = plugin is not None and _normalized_distribution_name(plugin.package) == _normalized_distribution_name(distribution)
        version_matches = plugin is not None and plugin.version == distribution_versions[distribution]
        plugin_records.append(
            {
                "plugin_id": plugin_id,
                "distribution": distribution,
                "entry_point_declared": entry_point_declared,
                "entry_point_target": declaration.target if declaration is not None else None,
                "entry_point_distribution": declaration.distribution if declaration is not None else None,
                "entry_point_distribution_matches": entry_point_distribution_matches,
                "entry_point_version": declaration.version if declaration is not None else None,
                "entry_point_version_matches": entry_point_version_matches,
                "loaded": loaded,
                "package_matches": package_matches,
                "version": plugin.version if plugin is not None else None,
                "version_matches": version_matches,
                "contribution_count": sum(contribution.plugin.id == plugin_id for contribution in contributions),
                "valid": (
                    entry_point_declared
                    and entry_point_distribution_matches
                    and entry_point_version_matches
                    and loaded
                    and package_matches
                    and version_matches
                ),
            }
        )
    ready = (
        all(bool(record["installed"]) for record in distribution_records)
        and all(bool(record["valid"]) for record in plugin_records)
        and not entry_point_discovery_errors
        and not failed_diagnostics
    )
    payload = {
        "schema": "taoryx.plugin-installation-check/v1",
        "profile": arguments.profile,
        "ready": ready,
        "distributions": distribution_records,
        "plugins": plugin_records,
        "entry_point_diagnostics": entry_point_discovery_errors,
        "diagnostics": failed_diagnostics,
    }
    if arguments.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0 if ready else 2

    print(f"Taoryx installation profile: {arguments.profile}")
    for record in distribution_records:
        marker = "OK" if record["installed"] else "FAIL"
        detail = record["version"] if record["version"] is not None else "not installed"
        print(f"[{marker:4}] distribution {record['distribution']}: {detail}")
    for record in plugin_records:
        marker = "OK" if record["valid"] else "FAIL"
        if not record["entry_point_declared"]:
            detail = "installed entry-point declaration is missing"
        elif not record["entry_point_distribution_matches"]:
            detail = f"entry point belongs to {record['entry_point_distribution']!r}, expected {record['distribution']!r}"
        elif not record["entry_point_version_matches"]:
            detail = f"entry-point version {record['entry_point_version']!r} does not match installed distribution"
        elif not record["loaded"]:
            detail = "entry point missing or failed to load"
        elif not record["package_matches"]:
            detail = f"advertises unexpected package metadata (expected {record['distribution']})"
        elif not record["version_matches"]:
            detail = f"advertised version {record['version']} does not match installed distribution"
        else:
            detail = f"{record['version']}; {record['contribution_count']} contribution(s)"
        print(f"[{marker:4}] plug-in {record['plugin_id']}: {detail}")
    for entry_point_diagnostic in entry_point_discovery_errors:
        print(f"[FAIL] entry-point declaration: {entry_point_diagnostic}")
    for failed_diagnostic in failed_diagnostics:
        print(f"[FAIL] {failed_diagnostic['plugin_id']}: {failed_diagnostic['message']}")
    if not ready:
        print("Installation is incomplete. Install the missing distributions from the release wheelhouse")
        print("or follow docs/INSTALLATION.md from a source checkout, then run this check again.")
        return 2
    print("Installation is ready; all required entry points loaded without source fallbacks.")
    return 0
    ####


def _normalized_distribution_name(name: str) -> str:
    """Compare distribution names using PEP 503 normalization."""

    return re.sub(r"[-_.]+", "-", name).lower()
    ####


def build_mission_composition_maturity_report(
    catalog: object,
    *,
    check_execution_witnesses: bool,
    execute_batch_witnesses: bool,
    execute_parity_witnesses: bool,
    results_directory: Path | None,
    retained_batch_results_directory: Path | None,
) -> dict[str, object]:
    """Lazily invoke the optional reference-model maturity reporter."""

    from taoryx.mission_composition_maturity import build_mission_composition_maturity_report as build_report

    return build_report(
        cast(Any, catalog),
        check_execution_witnesses=check_execution_witnesses,
        execute_batch_witnesses=execute_batch_witnesses,
        execute_parity_witnesses=execute_parity_witnesses,
        results_directory=results_directory,
        retained_batch_results_directory=retained_batch_results_directory,
    )
    ####


def build_vehicle_execution_witness_report(
    *,
    execute_batch: bool,
    family_ids: list[str] | None,
    witness_ids: list[str] | None,
    results_directory: Path | None,
) -> dict[str, object]:
    """Lazily run exact endpoint witnesses without widening their selected scope."""

    from taoryx.vehicle_execution_witnesses import validate_vehicle_execution_witnesses

    return validate_vehicle_execution_witnesses(
        execute_batch=execute_batch,
        family_ids=family_ids,
        witness_ids=witness_ids,
        retained_results_directory=results_directory,
    )
    ####


def _write_source_run_manifest(arguments: argparse.Namespace, report: object, artifact_path: Path | None) -> Path:
    """Write the common Simulation Runtime manifest for a file-oriented source run."""

    from taoryx.runtime.runner import RunReport

    if not isinstance(report, RunReport):
        raise TypeError("source run manifest requires a RunReport")
    artifact = report.artifacts[0] if report.artifacts else None
    scenario_id = artifact.scenario_identity if artifact is not None and artifact.scenario_identity else Path(arguments.problem).stem
    accepted_times = [time for item in report.artifacts for vehicle in item.vehicles.values() for time in vehicle.times]
    source_inputs = []
    missing_inputs: list[str] = []
    for path, role in [(arguments.problem, "problem"), *((path, "table") for path in arguments.tables)]:
        if Path(path).is_file():
            source_inputs.append(source_input_record(path, role=role))
        else:
            missing_inputs.append(str(path))
    termination = {
        "completed": report.status is SimulationRuntimeStatus.PASSED,
        "reason": report.results[0].stop_reason if report.results else "no_execution_result",
        "diagnostics": [item.code for item in report.diagnostics],
    }
    if missing_inputs:
        termination["missing_inputs"] = missing_inputs
    manifest = build_simulation_runtime_run_manifest(
        scenario_id=scenario_id,
        status=report.status,
        expected_disposition=report.status,
        operation="batch",
        fidelity="source_runtime",
        realization="source_runtime",
        source_inputs=tuple(source_inputs),
        runtime=default_runtime_identity(),
        integration={"profile": arguments.profile, "integrator": arguments.integrator, "max_steps": arguments.max_steps, "seed": arguments.seed},
        time={
            "requested_duration_s": None,
            "accepted_start_s": min(accepted_times) if accepted_times else None,
            "accepted_end_s": max(accepted_times) if accepted_times else None,
        },
        termination=termination,
        artifacts=artifact_inventory(arguments.output_dir),
        claim_boundary=(
            "This manifest identifies one source-runtime execution and its normalized artifacts. It does not prove "
            "numerical qualification, physical-effector behavior, historical fidelity, or mission capability."
        ),
        reproduction_command=("taoryx", "run", str(arguments.problem), *(str(path) for path in arguments.tables)),
    )
    return manifest.write_json(Path(arguments.output_dir) / "run-manifest.json")


def _daveml_command(arguments: argparse.Namespace) -> int:
    """Run fail-closed DAVE-ML family smoke verification."""

    from taoryx.trajectory.daveml_import import load_daveml_family_graph, load_daveml_family_import

    if arguments.daveml_command == "composite-smoke":
        return _a320_pseudo_smoke(arguments)
    if arguments.family == "a320_openap_3dof":
        return _a320_openap_smoke(arguments)

    sidecars = {
        "reference_f16_s119": Path("families/reference_f16_s119/plant/daveml-import.json"),
        "reference_hl20_mod_k": Path("families/reference_hl20_mod_k/plant/daveml-import.json"),
        "reference_nesc_two_stage_rocket": Path("families/reference_nesc_two_stage_rocket/plant/daveml-import.json"),
    }
    evidence = {
        "reference_f16_s119": Path("verification/daveml_f16_scenario_evidence.json"),
        "reference_hl20_mod_k": Path("verification/daveml_hl20_trim_evidence.json"),
        "reference_nesc_two_stage_rocket": Path("verification/daveml_nesc_replay_evidence.json"),
    }
    sidecar = sidecars[arguments.family]
    evidence_path = evidence[arguments.family]
    try:
        record = load_daveml_family_import(sidecar)
        roles = tuple(document.role for document in record.package.source_documents)
        for role in roles:
            load_daveml_family_graph(sidecar, role=role)
        report = json.loads(evidence_path.read_text(encoding="utf-8"))
        if report.get("family_id") != arguments.family:
            raise ValueError("evidence family does not match requested family")
        if report.get("status") not in {"pass", "verified"}:
            raise ValueError(f"evidence status is not promotable: {report.get('status')!r}")
        result = {
            "schema_version": "taoryx.daveml-cli-smoke/v1",
            "status": "verified",
            "family_id": arguments.family,
            "model_id": record.model_id,
            "roles_hash_verified": list(roles),
            "evidence": report,
        }
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f"error: daveml-smoke-failed: {error}")
        return 2
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


def _a320_pseudo_smoke(arguments: argparse.Namespace) -> int:
    """Run the bounded surrogate binding and authored-DAVE-ML smoke chain."""

    from taoryx.trajectory.a320_openap import A320OpenAPOperatingPoint
    from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel, A320Pseudo6DOFOperatingPoint

    try:
        model = A320Pseudo6DOFModel.from_repository()
        point = A320Pseudo6DOFOperatingPoint(
            A320OpenAPOperatingPoint(11000.0, 0.78, 60000.0),
            alpha_rad=0.04,
            beta_rad=0.02,
            aileron_rad=0.01,
            elevator_rad=-0.01,
            rudder_rad=0.01,
        )
        performance = model.evaluate(point)
        roundtrip = json.loads(Path("families/a320_openap_jsbsim_pseudo6dof/validation/roundtrip-report.json").read_text(encoding="utf-8"))
        runtime_qualification = json.loads(Path("families/a320_openap_jsbsim_pseudo6dof/validation/runtime-qualification.json").read_text(encoding="utf-8"))
        result = {
            "schema_version": "taoryx.daveml-cli-composite-smoke/v1",
            "status": "verified" if roundtrip.get("status") == "verified" and runtime_qualification.get("status") == "verified" else "failed",
            "family_id": arguments.family,
            "model_id": "a320-openap-jsbsim-pseudo6dof",
            "qualification_class": "surrogate_composite",
            "claim_boundary": "bounded surrogate binding and authored DAVE-ML channel smoke; not full 6-DOF qualification",
            "provenance": model.provenance,
            "result": performance.as_dict(),
            "rotational_derivatives": model.rotational_derivatives(point),
            "roundtrip": roundtrip,
            "runtime_qualification": runtime_qualification,
        }
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
        print(f"error: daveml-composite-smoke-failed: {error}")
        return 2
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "verified" else 2


def _a320_openap_smoke(arguments: argparse.Namespace) -> int:
    """Run the derived-exact A320 family-library evidence chain."""

    from taoryx.trajectory.a320_openap import A320OpenAPModel, A320OpenAPOperatingPoint

    try:
        model = A320OpenAPModel.from_repository()
        point = A320OpenAPOperatingPoint(altitude_m=11000.0, mach=0.78, mass_kg=60000.0)
        performance = model.evaluate(point)
        trim = model.trim_level_flight(point)
        tuning = model.tune_cruise_throttle(point)
        linearization = model.linearize_point_mass(point, trim)
        result = {
            "schema_version": "taoryx.daveml-cli-smoke/v1",
            "status": "verified" if trim.success and tuning.converged else "failed",
            "family_id": "a320_openap_3dof",
            "model_id": "a320-openap-3dof",
            "qualification_class": "derived_exact",
            "provenance": model.provenance,
            "performance": performance.as_dict(),
            "trim": {
                "success": trim.success,
                "state": dict(trim.state),
                "controls": {name: float(value) for name, value in trim.controls.items()},
                "residuals": dict(trim.residuals),
            },
            "tuning": {
                "status": str(tuning.status),
                "parameters": list(tuning.parameters),
                "objective": tuning.objective,
                "inequality_values": list(tuning.inequality_values),
            },
            "linearization": {
                "state_names": list(linearization.state_names),
                "control_names": list(linearization.control_names),
                "a_matrix": linearization.a_matrix.tolist(),
                "b_matrix": linearization.b_matrix.tolist(),
                "metadata": linearization.metadata_dict,
            },
            "objectives": model.score_level_flight_objectives(performance),
        }
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"error: daveml-a320-smoke-failed: {error}")
        return 2
    if arguments.output:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "verified" else 2


def _scenario_command(arguments: argparse.Namespace) -> int:
    """Handle Simulation Runtime discovery, manifest, bundle, and source compilation commands."""

    try:
        if arguments.scenario_command == "compile":
            return _compile_scenario(arguments)
        if arguments.scenario_command == "manifest" and arguments.scenario_manifest_command == "validate":
            manifest = read_simulation_runtime_run_manifest(arguments.path)
            payload = manifest.model_dump(mode="json", by_alias=True)
            if arguments.json:
                _print_json(payload)
            else:
                print(f"valid: {manifest.scenario_id} ({manifest.run_identity})")
            return 0
        catalog = load_simulation_runtime_catalog(arguments.catalog)
        if arguments.scenario_command == "list":
            rows = [_scenario_summary(item) for item in catalog.scenarios]
            if arguments.json:
                _print_json({"schema": catalog.schema_id, "schema_version": catalog.schema_version, "scenarios": rows})
            else:
                for row in rows:
                    print(f"{row['id']}: {row['title']} [{row['fidelity']}, {row['operation']}, expected={row['expected_disposition']}]")
            return 0
        if arguments.scenario_command == "search":
            rows = [_scenario_summary(item) for item in catalog.search(arguments.query)]
            if arguments.json:
                _print_json({"query": arguments.query, "matches": rows})
            else:
                for row in rows:
                    print(f"{row['id']}: {row['title']}")
            return 0
        if arguments.scenario_command == "show":
            scenario = catalog.find(arguments.identifier)
            if arguments.json:
                _print_json(scenario.as_dict())
            else:
                print(f"id: {scenario.id}")
                print(f"title: {scenario.title}")
                print(f"description: {scenario.description}")
                print(f"family: {scenario.family}")
                print(f"fidelity: {scenario.fidelity}")
                print(f"realization: {scenario.realization}")
                print(f"operation: {scenario.operation}")
                print(f"expected disposition: {scenario.expected_disposition.value}")
                print("inputs:")
                for item in scenario.inputs:
                    print(f"  - {item.path} ({item.role})")
                print("run command: " + " ".join(scenario.run_command))
                print(f"claim boundary: {scenario.claim_boundary}")
            return 0
        if arguments.scenario_command == "bundle":
            scenario = catalog.find(arguments.identifier)
            result = build_simulation_runtime_bundle(
                scenario,
                arguments.output_dir,
                run=not arguments.no_run,
                plots=not arguments.no_plots,
            )
            if arguments.json:
                _print_json(result)
            else:
                print(f"wrote Simulation Runtime bundle: {arguments.output_dir}")
                print(f"manifest: {result['manifest']}")
            return 0 if result["status"] in {SimulationRuntimeStatus.PASSED.value, SimulationRuntimeStatus.INCOMPLETE.value} else 2
        raise ValueError(f"unsupported scenario command: {arguments.scenario_command}")
    except (OSError, KeyError, TypeError, ValueError, SimulationRuntimeManifestCompatibilityError, RuntimeError) as error:
        print(f"error: scenario-failed: {error}")
        return 2
    ####


def _doctor_command(arguments: argparse.Namespace) -> int:
    """Run and print one catalog scenario's ordered diagnostic ladder."""

    try:
        scenario = load_simulation_runtime_catalog(arguments.catalog).find(arguments.identifier)
        report = doctor_scenario(scenario)
        if arguments.json:
            _print_json(report)
        else:
            print(f"scenario: {report['scenario_id']}")
            print(f"status: {report['status']} (expected disposition: {report['expected_disposition']})")
            for check in cast(list[dict[str, object]], report["checks"]):
                print(f"{check['status']}: {check['id']} — {check['message']}")
            for diagnostic in cast(list[dict[str, object]], report["diagnostics"]):
                print(f"diagnostic: {diagnostic['code']} at {diagnostic['location']}: {diagnostic['message']}")
        return 0 if report["status"] == SimulationRuntimeStatus.PASSED.value else 2
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"error: doctor-failed: {error}")
        return 2
    ####


def _scenario_summary(scenario: SimulationRuntimeScenario) -> dict[str, object]:
    """Return the compact list/search projection."""

    return {
        "id": scenario.id,
        "aliases": list(scenario.aliases),
        "title": scenario.title,
        "entrypoint": scenario.entrypoint,
        "family": scenario.family,
        "fidelity": scenario.fidelity,
        "realization": scenario.realization,
        "operation": scenario.operation,
        "expected_disposition": scenario.expected_disposition.value,
        "input_count": len(scenario.inputs),
        "claim_boundary": scenario.claim_boundary,
    }
    ####


def _compile_scenario(arguments: argparse.Namespace) -> int:
    """Compile source files into a deterministic scenario cache."""

    try:
        scenario = ScenarioCompiler().load_or_compile(
            arguments.output,
            arguments.problem,
            table_paths=tuple(arguments.tables),
            profile=arguments.profile,
            seed=arguments.seed,
            integrator=arguments.integrator,
        )
    except ScenarioCompileError as error:
        for diagnostic in error.diagnostics:
            location = diagnostic.location
            prefix = f"{location.path}:{location.line}: " if location else ""
            print(f"error: {prefix}{diagnostic.code}: {diagnostic.message}")
        return 2
    except (OSError, TypeError, ValueError) as error:
        print(f"error: scenario-compile-failed: {error}")
        return 2
    if arguments.json:
        print(scenario.model_dump_json(indent=2))
    else:
        print(f"compiled scenario {scenario.identity}")
        print(f"cache: {arguments.output}")
    return 0


def _trajectory_catalog(path: Path) -> FamilyCatalog:
    """Load the configured Alpha 2 family catalog for CLI commands."""

    return load_family_catalog(path)
    ####


def _print_json(payload: object, output: Path | None = None) -> None:
    """Print or write one deterministic JSON payload."""

    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if output is None:
        print(text, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    print(f"wrote {output}")
    ####


def _print_text(payload: str, output: Path | None = None) -> None:
    """Print or write one deterministic human-readable document."""

    text = payload if payload.endswith("\n") else payload + "\n"
    if output is None:
        print(text, end="")
        return
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    print(f"wrote {output}")
    ####


def _model_command(arguments: argparse.Namespace) -> int:
    """Drive every installed composer model through the common authoring seam."""

    try:
        plugins = discover_plugins()
        providers = plugins.build_mission_composition_provider_registry()
        campaigns = plugins.build_controller_tuning_campaign_registry()
        campaigns.validate_against(providers)
        if arguments.model_command == "endpoint-specs":
            _print_json(mission_workflow_endpoint_list(), arguments.output)
            return 0
        if arguments.model_command == "verify":
            report = verify_mission_workflow_endpoint(
                arguments.endpoint_id,
                execute=arguments.execute,
                plugins=plugins,
            )
            _print_json(report, arguments.output)
            return 0 if report["status"] == "pass" else 2
        if arguments.model_command == "overview":
            overview_payload = build_model_overview_catalog(
                plugins,
                providers,
                campaigns,
                provider_id=arguments.provider,
                model_id=arguments.model,
            )
            if arguments.format == "json":
                _print_json(overview_payload, arguments.output)
            else:
                _print_text(render_model_overview_markdown(overview_payload), arguments.output)
            return 0
        if arguments.model_command == "list":
            selected_providers = providers.providers
            if arguments.provider is not None:
                selected_providers = (providers.provider(arguments.provider),)
            payload: dict[str, object] = {
                "schema": "taoryx.model-authoring-catalog/v1",
                "plugin_catalog_fingerprint": plugins.fingerprint,
                "plugin_revisions": [item.public_dict() for item in plugins.plugin_revisions],
                "provider_catalog_fingerprint": providers.fingerprint,
                "providers": [
                    {
                        "metadata": provider.metadata.model_dump(mode="json", by_alias=True),
                        "revision": providers.provider_revision(provider.metadata.id),
                        "models": [
                            {
                                "id": model.id,
                                "version": model.version,
                                "metadata_fingerprint": model.metadata_fingerprint,
                                "revision": model.revision_public_dict(),
                                "display_name": model.presentation.display_name,
                                "family_id": model.family_id,
                                "physical_family": model.physical_family,
                                "model_kind": model.model_kind,
                                "status": model.status,
                                "fidelities": [item.model_dump(mode="json") for item in model.fidelities],
                                "realizations": [
                                    {
                                        "id": item.id,
                                        "status": item.status,
                                        "fidelity_aliases": list(item.fidelity_aliases),
                                        "input_realization": item.input_realization,
                                        "control_status": item.controls.status,
                                    }
                                    for item in model.realizations
                                ],
                                "mission_template_ids": [item.id for item in model.mission_templates],
                                "controller_tuning_campaign_ids": [
                                    item.id for item in campaigns.registrations if item.provider_id == provider.metadata.id and item.model_id == model.id
                                ],
                            }
                            for model in provider.list_models()
                        ],
                    }
                    for provider in selected_providers
                ],
                "workflow": {
                    "installation_check": "taoryx plugins check --profile models",
                    "next": "taoryx model plan <provider-id> <model-id>",
                },
                "claim_boundary": (
                    "This inventory reports installed advertisements and campaign registrations; "
                    "it does not imply that every fidelity or mission operation is executable."
                ),
            }
            _print_json(payload, arguments.output)
            return 0
        if arguments.model_command == "assess":
            family_adapters = plugins.build_family_adapter_registry() if plugins.records("family_adapter") else None
            local_controller_screens = (
                plugins.build_local_controller_screen_advertisement_registry() if plugins.records("local_controller_screen_advertisement") else None
            )
            payload = build_model_automation_assessment(
                providers,
                campaigns,
                family_adapters=family_adapters,
                local_controller_screens=local_controller_screens,
                provider_id=arguments.provider,
            )
            if arguments.summary:
                payload = build_model_automation_readiness_summary(payload)
            _print_json(payload, arguments.output)
            return 0
        if arguments.model_command == "plan":
            family_adapters = plugins.build_family_adapter_registry() if plugins.records("family_adapter") else None
            local_controller_screens = (
                plugins.build_local_controller_screen_advertisement_registry() if plugins.records("local_controller_screen_advertisement") else None
            )
            payload = build_model_authoring_plan(
                providers,
                campaigns,
                arguments.provider_id,
                arguments.model_id,
                family_adapters=family_adapters,
                local_controller_screens=local_controller_screens,
                fidelity=arguments.fidelity,
                realization_id=arguments.realization,
                mission_template_id=arguments.mission,
            )
            _print_json(payload, arguments.output)
            return 0
        if arguments.model_command == "scaffold":
            draft = scaffold_model_authoring_draft(
                providers,
                arguments.provider_id,
                arguments.model_id,
                fidelity=arguments.fidelity,
                realization_id=arguments.realization,
                mission_template_id=arguments.mission,
                draft_id=arguments.draft_id,
                configuration_id=arguments.configuration_id,
            )
            destination = write_model_authoring_draft(draft, arguments.output)
            _print_json(
                {
                    "schema": "taoryx.model-authoring-scaffold-result/v1",
                    "output": str(destination),
                    "draft_id": draft.draft_id,
                    "configuration_id": draft.configuration_id,
                    "status": "inputs_required" if draft.unresolved_inputs else "complete",
                    "unresolved_inputs": list(draft.unresolved_inputs),
                    "next": f"taoryx model compile {destination}",
                }
            )
            return 0
        if arguments.model_command == "compile":
            draft = load_model_authoring_draft(arguments.draft)
            prepared = compile_model_authoring_draft(providers, draft)
            _print_json(prepared.model_dump(mode="json", by_alias=True), arguments.output)
            return 0
        if arguments.model_command == "run":
            prepared = PreparedTrajectoryConfiguration.model_validate_json(arguments.prepared.read_text(encoding="utf-8"))
            response = run_prepared_mission_composition(
                providers,
                arguments.provider_id,
                prepared,
                request_id=arguments.request_id,
                output=MissionCompositionOutputSelection(
                    mode=arguments.output_mode,
                    cadence_s=arguments.cadence_s,
                    channels=tuple(arguments.channel),
                    telemetry_groups=tuple(arguments.telemetry_group),
                    include_events=not arguments.no_events,
                    include_segments=not arguments.no_segments,
                    include_spawned_objects=not arguments.no_spawned_objects,
                    maximum_samples_per_object=arguments.maximum_samples_per_object,
                    maximum_objects=arguments.maximum_objects,
                ),
            )
            _print_json(response.model_dump(mode="json", by_alias=True), arguments.output)
            return 0 if response.kind == "trajectory" else 2
        if arguments.model_command == "tune":
            registration = _select_model_tuning_campaign(campaigns, arguments)
            selection = resolve_model_authoring_selection(
                providers,
                arguments.provider_id,
                arguments.model_id,
                fidelity=arguments.fidelity or registration.fidelity,
                realization_id=(arguments.realization or (registration.realization_ids[0] if len(registration.realization_ids) == 1 else None)),
                mission_template_id=(arguments.mission or (registration.mission_template_ids[0] if len(registration.mission_template_ids) == 1 else None)),
                allow_blocked_local_design=True,
            )
            selected_realization = selection.realization.id if selection.realization is not None else None
            selected_mission = selection.mission.id if selection.mission is not None else None
            if selection.model.family_id != registration.family_id or not registration.matches(
                provider_id=arguments.provider_id,
                model_id=arguments.model_id,
                fidelity=selection.fidelity,
                realization_id=selected_realization,
                mission_template_id=selected_mission,
            ):
                raise ModelAuthoringError(
                    "campaign-selection-mismatch",
                    f"campaign {registration.id!r} does not apply to the resolved model selection",
                    path="campaign",
                )
            cached = registration.run_cached(
                None if arguments.no_cache else arguments.cache_dir,
                context_fingerprint=plugins.fingerprint,
            )
            _print_json(
                {
                    "schema": "taoryx.model-tuning-result/v1",
                    "selection": selection.public_dict(),
                    "registration": registration.public_dict(),
                    "cache": {
                        "key": cached.cache_key,
                        "hit": cached.cache_hit,
                        "path": str(cached.cache_path) if cached.cache_path is not None else None,
                    },
                    "report": cached.payload,
                },
                arguments.output,
            )
            return 0 if cached.payload.get("status") == "candidate_ready" else 2
        raise ValueError(f"unknown model command {arguments.model_command!r}")
    except ModuleNotFoundError as error:
        print(f"error: model-tool-dependency-missing: {error}; install the model packages plus the declared numerical extras")
        return 2
    except (KeyError, ModelAuthoringError, OSError, PluginError, TypeError, ValueError, RuntimeError) as error:
        print(f"error: model-tool-failed: {error}")
        return 2
    ####


def _select_model_tuning_campaign(
    campaigns: ControllerTuningCampaignRegistry,
    arguments: argparse.Namespace,
) -> ControllerTuningCampaignRegistration:
    """Resolve an explicit campaign or the only registration for a model."""

    if arguments.campaign is not None:
        registration = campaigns.registration(arguments.campaign)
        if arguments.provider_id not in (registration.provider_id, *registration.provider_aliases) or registration.model_id != arguments.model_id:
            raise ModelAuthoringError(
                "campaign-model-mismatch",
                (f"campaign targets {registration.provider_id!r}/{registration.model_id!r}, not {arguments.provider_id!r}/{arguments.model_id!r}"),
                path="campaign",
            )
        return registration
    matches = tuple(
        item
        for item in campaigns.registrations
        if arguments.provider_id in (item.provider_id, *item.provider_aliases) and item.model_id == arguments.model_id
    )
    if len(matches) != 1:
        raise ModelAuthoringError(
            "campaign-selection-required",
            f"model has registered campaigns {[item.id for item in matches]!r}; select one explicitly",
            path="campaign",
        )
    return matches[0]
    ####


def _catalog_command(arguments: argparse.Namespace) -> int:
    """Handle Alpha 2 catalog inspection."""

    try:
        catalog = _trajectory_catalog(arguments.catalog)
        if arguments.catalog_command == "list" and arguments.kind == "families":
            _print_json(
                [
                    {
                        "family_id": family.family_id,
                        "version": family.version,
                        "display_name": family.display_name,
                        "fidelities": list(family.fidelities),
                    }
                    for family in catalog.families
                ]
            )
        return 0
    except (OSError, KeyError, ResolutionError, TypeError, ValueError) as error:
        print(f"error: catalog-failed: {error}")
        return 2
    ####


def _reachability_command(arguments: argparse.Namespace) -> int:
    """Delegate optional reachability operations through the typed provider registry."""

    try:
        catalog = discover_plugins()
        provider = catalog.build_reachability_provider_registry().provider("taoryx.reachability.workbench")
    except KeyError:
        print("error: reachability-plugin-required: install taoryx-reachability to use reachability commands")
        return 2
    except PluginError as error:
        print(f"error: reachability-plugin-load-failed: {error}")
        return 2
    return provider.run_cli(arguments)
    ####


def _family_command(arguments: argparse.Namespace) -> int:
    """Handle Alpha 2 family inspection and schema export."""

    try:
        family = _trajectory_catalog(arguments.catalog).family(arguments.family_id)
        if arguments.family_command == "inspect":
            _print_json(family.model_dump(mode="json"))
        else:
            _print_json(
                {
                    "family_id": family.family_id,
                    "version": family.version,
                    "parameters": [item.model_dump(mode="json") for item in family.parameters],
                    "controls": [item.model_dump(mode="json") for item in family.controls],
                    "observations": [item.model_dump(mode="json") for item in family.observations],
                }
            )
        return 0
    except (OSError, KeyError, ResolutionError, TypeError, ValueError) as error:
        print(f"error: family-failed: {error}")
        return 2
    ####


def _vehicle_command(arguments: argparse.Namespace) -> int:
    """Handle user-facing vehicle/fidelity/trajectory composition discovery."""

    try:
        if arguments.vehicle_command == "intake":
            return _vehicle_intake_command(arguments)
        if arguments.vehicle_command == "integration":
            return _vehicle_integration_command(arguments)
        catalog = load_resolved_vehicle_composition_catalog()
        if arguments.vehicle_command == "compose":
            compiled = compile_vehicle_composition(load_vehicle_composition_request(arguments.request), catalog=catalog)
            if arguments.output is not None:
                compiled.write_json(arguments.output)
            _print_json(compiled.model_dump(mode="json", by_alias=True))
            return 0
        if arguments.vehicle_command == "mission":
            if arguments.vehicle_mission_command == "inspect":
                vehicle = catalog.vehicle(arguments.identifier)
                descriptor = vehicle.as_dict()
                missions = descriptor.get("mission_templates")
                if not isinstance(missions, list):
                    raise VehicleCompositionError("invalid-registry", "mission templates are not a list")
                matches = [item for item in missions if isinstance(item, Mapping) and item.get("id") == arguments.mission_id]
                if len(matches) != 1:
                    raise VehicleCompositionError(
                        "unknown-mission",
                        f"unknown mission {arguments.mission_id!r}",
                        field="mission_id",
                    )
                mission = matches[0]
                initialization_ids = mission.get("initialization_contracts")
                segment_ids = mission.get("segment_sequence")
                if not isinstance(initialization_ids, list) or not isinstance(segment_ids, list):
                    raise VehicleCompositionError("invalid-registry", "mission has invalid initialization or segment references")
                initializations = descriptor.get("initialization_contracts")
                segments = descriptor.get("segment_contracts")
                fidelities = descriptor.get("fidelities")
                if not isinstance(initializations, list) or not isinstance(segments, list) or not isinstance(fidelities, Mapping):
                    raise VehicleCompositionError("invalid-registry", "vehicle has invalid mission-contract records")
                compatible_fidelities = mission.get("compatible_fidelities")
                if not isinstance(compatible_fidelities, list):
                    raise VehicleCompositionError("invalid-registry", "mission has invalid fidelity references")
                _print_json(
                    {
                        "schema": "taoryx.vehicle-mission-inspection/v1alpha1",
                        "vehicle_id": descriptor["vehicle_id"],
                        "family_id": descriptor["family_id"],
                        "mission_template": mission,
                        "initialization_contracts": [item for item in initializations if isinstance(item, Mapping) and item.get("id") in initialization_ids],
                        "segment_contracts": [item for item in segments if isinstance(item, Mapping) and item.get("id") in segment_ids],
                        "fidelity_contracts": {fidelity: fidelities[fidelity] for fidelity in compatible_fidelities if fidelity in fidelities},
                        "claim_boundary": (
                            "Inspection exposes the declared semantic mission contract only. It does not choose "
                            "authoring values, bind a native adapter, execute a plant, or qualify a result."
                        ),
                    }
                )
                return 0
            if arguments.vehicle_mission_command == "create":
                kit = catalog.vehicle(arguments.identifier).authoring_kit_dict(arguments.mission_id, arguments.fidelity)
                _print_json(kit, arguments.output)
                return 0
            if arguments.vehicle_mission_command == "validate":
                compiled = compile_vehicle_composition(load_vehicle_composition_request(arguments.request), catalog=catalog)
                if arguments.compiled_output is not None:
                    compiled.write_json(arguments.compiled_output)
                interface = resolve_vehicle_composition_interface_contract(compiled, catalog=catalog)
                findings = validate_vehicle_interface_contract(interface)
                preflight = preflight_vehicle_composition(compiled)
                lowering = lower_vehicle_composition(compiled)
                mission_graph = compiled.mission_graph
                if mission_graph is None:
                    raise VehicleCompositionError("missing-mission-graph", "compiled composition is missing its mission graph")
                _print_json(
                    {
                        "schema": "taoryx.vehicle-mission-validation/v1alpha1",
                        "request": str(arguments.request),
                        "composition": {
                            "id": compiled.id,
                            "identity_sha256": compiled.identity_sha256,
                            "family_id": compiled.family_id,
                            "mission_id": compiled.mission,
                            "fidelity": compiled.fidelity,
                            "mission_graph": mission_graph.model_dump(mode="json", by_alias=True),
                        },
                        "semantic_validation": "pass",
                        "interface_validation": {
                            "status": "pass" if not findings else "fail",
                            "findings": list(findings),
                        },
                        "runtime_readiness": {
                            "preflight_status": preflight.status,
                            "preflight": preflight.as_dict(),
                            "lowering_status": lowering.status,
                            "execution_binding": lowering.execution_binding,
                        },
                        "claim_boundary": (
                            "Mission validation proves only immutable semantic compilation and interface conformance. "
                            "It projects the selected preflight advertisement and lowering disposition without running "
                            "the model. A non-ready adapter, missing batch factory, or failed mission execution remains "
                            "visible and cannot be promoted by this command."
                        ),
                    }
                )
                return 0 if not findings else 2
            raise ValueError(f"unknown vehicle mission command {arguments.vehicle_mission_command!r}")
        if arguments.vehicle_command == "variant":
            compiled = compile_vehicle_composition(load_vehicle_composition_request(arguments.request), catalog=catalog)
            if arguments.vehicle_variant_command == "resolve":
                compiled.write_json(arguments.output)
                _print_json(
                    {
                        "schema": "taoryx.vehicle-variant-resolution/v1alpha1",
                        "request": str(arguments.request),
                        "composition_id": compiled.id,
                        "composition_identity_sha256": compiled.identity_sha256,
                        "variant": compiled.variant.model_dump(mode="json"),
                        "compiled_composition": str(arguments.output),
                        "claim_boundary": (
                            "Resolution creates an immutable semantic composition. It does not execute a runtime, "
                            "retrim a plant, or requalify the selected vehicle."
                        ),
                    }
                )
                return 0
            if arguments.vehicle_variant_command == "validate":
                _print_json(
                    {
                        "schema": "taoryx.vehicle-variant-validation/v1alpha1",
                        "request": str(arguments.request),
                        "composition_id": compiled.id,
                        "composition_identity_sha256": compiled.identity_sha256,
                        "variant": compiled.variant.model_dump(mode="json"),
                        "resulting_initialization_inputs": {
                            identifier: value.model_dump(mode="json") for identifier, value in compiled.initialization.inputs.items()
                        },
                        "status": "valid",
                        "claim_boundary": (
                            "Validation proves bounded semantic resolution and named runtime binding only. It does not "
                            "execute the runtime, establish trim, or promote qualification."
                        ),
                    }
                )
                return 0
            raise ValueError(f"unknown vehicle variant command {arguments.vehicle_variant_command!r}")
        if arguments.vehicle_command == "validate-examples":
            report = _validate_vehicle_composition_examples(
                arguments.directory,
                catalog=catalog,
                require_preflight=arguments.require_preflight,
            )
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "interface-composition":
            composition = load_compiled_vehicle_composition(arguments.composition)
            contract = resolve_vehicle_composition_interface_contract(composition, catalog=catalog)
            findings = validate_vehicle_interface_contract(contract)
            _print_json(
                {
                    **contract.as_dict(),
                    "composition_id": composition.id,
                    "composition_identity_sha256": composition.identity_sha256,
                    "validation": {
                        "status": "pass" if not findings else "fail",
                        "findings": list(findings),
                    },
                }
            )
            return 0 if not findings else 2
        if arguments.vehicle_command == "lower":
            _print_json(lower_vehicle_composition(load_compiled_vehicle_composition(arguments.composition)).as_dict())
            return 0
        if arguments.vehicle_command == "preflight":
            result = preflight_vehicle_composition(load_compiled_vehicle_composition(arguments.composition))
            _print_json(result.as_dict())
            return 0 if result.status == "translation_ready" else 2
        if arguments.vehicle_command == "materialize":
            composition = load_compiled_vehicle_composition(arguments.composition)
            materialized = materialize_powered_fixed_wing_composition(composition, arguments.output_dir)
            _print_json(
                {
                    "schema": "taoryx.vehicle-language-backed-materialization/v1alpha1",
                    "composition_id": composition.id,
                    "composition_identity_sha256": composition.identity_sha256,
                    "source_mission_id": materialized.source_mission_id,
                    "materialized_mission_id": materialized.materialized_mission_id,
                    "proposal": materialized.proposal.manifest(),
                    "inputs": {
                        "problem": str(materialized.problem),
                        "mission_config": str(materialized.mission_config),
                        "racetrack_config": str(materialized.racetrack_config),
                    },
                    "claim_boundary": (
                        "Materialization writes a route and independent-evaluation inputs matching preflight. "
                        "It does not bind a runtime adapter, execute a controller, or qualify the vehicle."
                    ),
                }
            )
            return 0
        if arguments.vehicle_command == "run":
            if arguments.max_steps is not None and arguments.max_steps <= 0:
                raise ValueError("--max-steps must be positive")
            composition = load_compiled_vehicle_composition(arguments.composition)
            batch_execution = execute_vehicle_composition_batch(
                composition,
                arguments.output_dir,
                max_steps=arguments.max_steps,
            )
            payload = _finalize_vehicle_batch_execution_payload(
                batch_execution.as_dict(),
                output_dir=arguments.output_dir,
                composition=composition,
                binding=batch_execution.binding,
                composition_path=arguments.composition,
                max_steps=arguments.max_steps,
                catalog=catalog,
            )
            _print_json(payload)
            return 0 if batch_execution.passed else 1
        if arguments.vehicle_command == "replay-policy":
            from taoryx.composition_policy import replay_composition_policy_trace_file

            composition = load_compiled_vehicle_composition(arguments.composition)
            replay_report = replay_composition_policy_trace_file(composition, arguments.trace)
            _print_json(replay_report.as_dict(), arguments.output)
            return 0 if replay_report.status == "pass" else 1
        if arguments.vehicle_command == "batch-episode-parity":
            from taoryx.batch_episode_parity_dispatch import verify_serialized_declared_batch_episode_parity

            composition = load_compiled_vehicle_composition(arguments.composition)
            trace_payload = json.loads(arguments.trace.read_text(encoding="utf-8"))
            if not isinstance(trace_payload, Mapping):
                raise ValueError("policy trace JSON must contain one object")
            parity_report = verify_serialized_declared_batch_episode_parity(composition, trace_payload)
            _print_json(parity_report.as_dict(), arguments.output)
            return 0 if parity_report.status == "pass" else 1
        if arguments.vehicle_command == "episode-info":
            from taoryx.composition_episode import open_vehicle_composition_episode

            composition = load_compiled_vehicle_composition(arguments.composition)
            binding = resolve_vehicle_execution_binding(composition, "episode")
            episode = open_vehicle_composition_episode(composition, seed=arguments.seed)
            try:
                initial_truth = episode.reset(seed=arguments.seed)
                initial_status = episode.status_frame()
                initial_observation = episode.observe_frame(composition.observation.profile_id)
                _print_json(
                    {
                        "schema": "taoryx.vehicle-composition-episode-info/v1alpha1",
                        "composition_id": composition.id,
                        "composition_identity_sha256": composition.identity_sha256,
                        "execution_binding": binding.model_dump(mode="json"),
                        "claim_boundary": episode.claim_boundary,
                        "interface": {
                            "id": episode.interface_contract.id,
                            "fingerprint_sha256": episode.interface_contract.fingerprint,
                        },
                        "action_schema": [channel.as_dict() for channel in episode.action_schema],
                        "observation_schema": [channel.as_dict() for channel in episode.observation_schema],
                        "initial_truth": initial_truth.as_dict(),
                        "initial_status": initial_status.as_dict(),
                        "initial_observation": initial_observation.as_dict(),
                    }
                )
            finally:
                episode.close()
            return 0
        if arguments.vehicle_command == "result":
            evaluation_path = arguments.output_dir / "evaluation.json"
            if not evaluation_path.is_file():
                result_catalog = index_composition_results(arguments.output_dir)
                catalog_records = result_catalog.get("records")
                if not isinstance(catalog_records, list):
                    raise ValueError("local controller screen catalog has invalid record payload")
                screen_records = [
                    record
                    for record in catalog_records
                    if isinstance(record, Mapping)
                    and record.get("record_kind") == "local_controller_screen"
                    and record.get("status") == "valid"
                    and record.get("output_directory") == "."
                ]
                if result_catalog["status"] != "pass" or len(screen_records) != 1:
                    raise ValueError("result directory has no normalized evaluation.json and does not contain exactly one valid local controller screen result")
                screen_record = screen_records[0]
                if arguments.composition is not None:
                    composition = load_compiled_vehicle_composition(arguments.composition)
                    provenance = screen_record.get("composition_provenance")
                    if not isinstance(provenance, Mapping):
                        raise ValueError("local controller screen has no composition provenance")
                    if provenance.get("composition_id") != composition.id:
                        raise ValueError("local controller screen composition identity does not match the supplied composition")
                    if provenance.get("composition_identity_sha256") != composition.identity_sha256:
                        raise ValueError("local controller screen composition fingerprint does not match the supplied composition")
                _print_json(
                    {
                        "schema": "taoryx.vehicle-composition-result/v1alpha1",
                        "output_directory": str(arguments.output_dir),
                        "record_kind": "local_controller_screen",
                        "result": screen_record,
                        "claim_boundary": (
                            "This is a verified local controller-screen record. It is not a normalized mission "
                            "evaluation, terminal result, physical-effector allocation result, or qualification."
                        ),
                    }
                )
                return 0
            payload = json.loads(evaluation_path.read_text(encoding="utf-8"))
            evaluation = TrajectoryEvaluation.model_validate(payload)
            composition_identity_sha256: str | None = None
            if arguments.composition is not None:
                composition = load_compiled_vehicle_composition(arguments.composition)
                composition_identity_sha256 = composition.identity_sha256
                if evaluation.scenario_id != composition.id:
                    raise ValueError(
                        f"evaluation scenario identity does not match the supplied compiled composition: {evaluation.scenario_id!r} != {composition.id!r}"
                    )
                if evaluation.scenario_contract_sha256 != composition.identity_sha256:
                    raise ValueError("evaluation composition fingerprint does not match the supplied compiled composition")
            artifact_names = (
                "execution.json",
                "nonlinear_validation.json",
                "mission_graph_execution.json",
                "objective_report.json",
                "status_trace.json",
                "vehicle_interface.json",
                "telemetry.csv",
                "truth_telemetry.csv",
            )
            _print_json(
                {
                    "schema": "taoryx.vehicle-composition-result/v1alpha1",
                    "output_directory": str(arguments.output_dir),
                    "composition_identity_sha256": composition_identity_sha256,
                    "evaluation": evaluation.as_dict(),
                    "artifacts": {name: str(arguments.output_dir / name) for name in artifact_names if (arguments.output_dir / name).is_file()},
                }
            )
            return 0
        if arguments.vehicle_command == "results":
            report = index_composition_results(arguments.directory)
            _print_json(report)
            return 0 if report["status"] in {"pass", "empty"} else 2
        if arguments.vehicle_command == "release-catalog":
            report = write_composition_release_catalog(arguments.directory, arguments.output)
            _print_json(report)
            return 0
        if arguments.vehicle_command == "catalog":
            _print_json(catalog.as_dict(detail=arguments.detail))
            return 0
        if arguments.vehicle_command == "list":
            _print_json(catalog.list_dict())
            return 0
        if arguments.vehicle_command == "interface-report":
            report = build_vehicle_interface_catalog_report(catalog)
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "topology-report":
            report = build_vehicle_composition_topology_report(catalog)
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "semantic-preflight-handler-report":
            report = build_semantic_preflight_handler_report(catalog)
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "maturity-report":
            if arguments.retain_batch_results_dir is not None and not arguments.execute_batch_witnesses:
                raise ValueError("vehicle maturity-report --retain-batch-results-dir requires --execute-batch-witnesses")
            if arguments.retain_batch_results_dir is not None and arguments.results_dir is not None:
                raise ValueError("vehicle maturity-report accepts either --results-dir or --retain-batch-results-dir, not both")
            report = build_mission_composition_maturity_report(
                catalog,
                check_execution_witnesses=arguments.check_execution_witnesses,
                execute_batch_witnesses=arguments.execute_batch_witnesses,
                execute_parity_witnesses=arguments.execute_parity_witnesses,
                results_directory=arguments.results_dir,
                retained_batch_results_directory=arguments.retain_batch_results_dir,
            )
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "witness-report":
            report = build_vehicle_execution_witness_report(
                execute_batch=arguments.execute_batch,
                family_ids=arguments.family,
                witness_ids=arguments.witness,
                results_directory=arguments.results_dir,
            )
            _print_json(report)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "endpoint-specs":
            _print_json(vehicle_endpoint_spec_list())
            return 0
        if arguments.vehicle_command == "verify":
            if arguments.results_dir is not None and not arguments.execute:
                raise ValueError("vehicle verify --results-dir requires --execute")
            report = verify_vehicle_endpoint(
                arguments.endpoint_id,
                execute=arguments.execute,
                tune=arguments.tune,
                cache_dir=None if arguments.no_cache else arguments.cache_dir,
                results_dir=arguments.results_dir,
                composition_catalog=catalog,
            )
            _print_json(report, arguments.output)
            return 0 if report["status"] == "pass" else 2
        if arguments.vehicle_command == "interface":
            contract = resolve_vehicle_interface_contract(arguments.identifier, arguments.fidelity, catalog=catalog)
            findings = validate_vehicle_interface_contract(contract)
            _print_json(
                {
                    **contract.as_dict(),
                    "validation": {
                        "status": "pass" if not findings else "fail",
                        "findings": list(findings),
                    },
                }
            )
            return 0 if not findings else 2
        if arguments.vehicle_command == "parameters":
            _print_json(catalog.vehicle(arguments.identifier).parameter_dict(arguments.scope))
            return 0
        if arguments.vehicle_command == "authoring":
            _print_json(catalog.vehicle(arguments.identifier).authoring_worklist_dict())
            return 0
        if arguments.vehicle_command == "authoring-template":
            _print_json(catalog.vehicle(arguments.identifier).authoring_kit_dict(arguments.mission_id, arguments.fidelity))
            return 0
        if arguments.vehicle_command == "authoring-all":
            _print_json(catalog.authoring_worklist_dict())
            return 0
        payload = catalog.vehicle(arguments.identifier).as_dict()
        if arguments.vehicle_command in {"inspect", "describe"}:
            _print_json(payload)
        elif arguments.vehicle_command == "missions":
            _print_json(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "mission_templates": payload["mission_templates"],
                }
            )
        elif arguments.vehicle_command == "segment":
            segment_contracts = payload["segment_contracts"]
            if not isinstance(segment_contracts, list):
                raise VehicleCompositionError("invalid-registry", "segment contracts are not a list")
            matches = [item for item in segment_contracts if isinstance(item, Mapping) and item.get("id") == arguments.segment_id]
            if len(matches) != 1:
                raise VehicleCompositionError("unknown-segment", f"unknown segment {arguments.segment_id!r}", field="segment_id")
            _print_json(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "segment_contract": matches[0],
                }
            )
        elif arguments.vehicle_command == "endpoints":
            _print_json(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "execution_bindings": payload["execution_bindings"],
                    "batch_episode_parity": payload["batch_episode_parity"],
                }
            )
        else:
            field = {
                "initialization": "initialization_contracts",
                "segments": "segment_contracts",
                "missions": "mission_templates",
            }[arguments.kind]
            _print_json(
                {
                    "vehicle_id": payload["vehicle_id"],
                    "family_id": payload["family_id"],
                    "fidelities": payload["fidelities"],
                    field: payload[field],
                }
            )
        return 0
    except (OSError, KeyError, TypeError, ValueError, VehicleCompositionError) as error:
        print(f"error: vehicle-registry-failed: {error}")
        return 2
    ####


def _vehicle_intake_command(arguments: argparse.Namespace) -> int:
    """Export the existing intake generator through the Mission Composition CLI.

    The result selects a declared topology strategy but remains a
    non-promotable intake artifact. It never fabricates source mappings,
    control authority, trim, or a registry entry.
    """

    try:
        if arguments.vehicle_intake_mode == "existing-family":
            payload = build_existing_family_intake_blueprint(
                ExistingFamilyIntakeRequest(
                    family_id=arguments.family_id,
                    physical_family=arguments.physical_family,
                    mission_overlay=arguments.mission_overlay,
                    adapter_id=arguments.adapter_id,
                    strategy_id=arguments.strategy_id,
                )
            ).as_dict()
        else:
            payload = build_new_topology_intake_scaffold(
                NewTopologyIntakeRequest(
                    family_id=arguments.family_id,
                    physical_family=arguments.physical_family,
                    proposed_strategy_id=arguments.strategy_id,
                    topology_summary=arguments.topology_summary,
                )
            ).as_dict()
        _print_json(payload, arguments.output)
        return 0
    except (OSError, StrategySelectionError, ValueError) as error:
        print(f"error: vehicle-intake-failed: {error}")
        return 2
    ####


def _vehicle_integration_command(arguments: argparse.Namespace) -> int:
    """Expose source-family onboarding gates through the Mission Composition CLI.

    This command intentionally reuses the provider-neutral integration records
    rather than treating a composition-registry entry as proof that a source
    plant is trim-ready, runnable, or qualified.  It makes the missing work
    visible to a vehicle author at the same public surface used for intake and
    composition discovery.
    """

    from taoryx.vehicle_integration_pipeline import (
        validate_all_vehicle_integration_pipelines,
        validate_vehicle_integration_pipeline,
        write_vehicle_integration_packet,
    )

    try:
        if arguments.vehicle_integration_command == "readiness":
            reports = validate_all_vehicle_integration_readiness() if arguments.family == "all" else (validate_vehicle_integration_readiness(arguments.family),)
            payload: dict[str, object] = {
                "schema": "taoryx.vehicle-integration-command/v1alpha1",
                "command": "readiness",
                "family": arguments.family,
                "claim_boundary": (
                    "Metadata readiness only. This does not execute a source plant, establish trim, "
                    "bind controls, or promote any fidelity or qualification claim."
                ),
                "reports": [report.as_dict() for report in reports],
            }
            _print_json(payload, arguments.output)
            return 0 if all(not report.errors for report in reports) else 1

        pipeline_reports = (
            validate_all_vehicle_integration_pipelines() if arguments.family == "all" else (validate_vehicle_integration_pipeline(arguments.family),)
        )
        if arguments.packet_dir is not None:
            for report in pipeline_reports:
                packet_path = arguments.packet_dir / report.family_id if arguments.family == "all" else arguments.packet_dir
                write_vehicle_integration_packet(
                    report.family_id,
                    packet_path,
                    include_source_package=not arguments.exclude_source_package,
                )
        payload = {
            "schema": "taoryx.vehicle-integration-command/v1alpha1",
            "command": "pipeline",
            "family": arguments.family,
            "claim_boundary": (
                "Staged provider-neutral integration evidence only. It does not execute a mission, prove physical allocation, or qualify a vehicle family."
            ),
            "reports": [report.as_dict() for report in pipeline_reports],
        }
        _print_json(payload, arguments.output)
        return 0 if arguments.allow_blocked or all(not report.blockers for report in pipeline_reports) else 1
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"error: vehicle-integration-failed: {error}")
        return 2
    ####


def _validate_vehicle_composition_examples(
    directory: Path,
    *,
    catalog: ResolvedVehicleCompositionCatalog,
    require_preflight: bool,
) -> dict[str, object]:
    """Return a deterministic authoring report for all composition fixtures.

    This validates only semantic composition and declared binding readiness.
    It intentionally does not run simulations, treat a translation-ready
    adapter as qualification, or suppress a preflight gap found in a witness.
    """

    if not directory.is_dir():
        raise ValueError(f"composition example directory does not exist: {directory}")
    paths = tuple(sorted((*directory.glob("*.yaml"), *directory.glob("*.yml"), *directory.glob("*.json"))))
    if not paths:
        raise ValueError(f"composition example directory contains no YAML or JSON requests: {directory}")
    records: list[dict[str, object]] = []
    all_passed = True
    for path in paths:
        try:
            composition = compile_vehicle_composition(load_vehicle_composition_request(path), catalog=catalog)
            contract = resolve_vehicle_composition_interface_contract(composition, catalog=catalog)
            interface_findings = validate_vehicle_interface_contract(contract)
            lowering = lower_vehicle_composition(composition)
            preflight = preflight_vehicle_composition(composition)
            ready = preflight.status == "translation_ready"
            runtime_bound = lowering.status in {"adapter_bound", "factory_bound"}
            record_passed = not interface_findings and (not require_preflight or (ready and runtime_bound))
            all_passed = all_passed and record_passed
            records.append(
                {
                    "path": str(path),
                    "composition_id": composition.id,
                    "composition_identity_sha256": composition.identity_sha256,
                    "family_id": composition.family_id,
                    "fidelity": composition.fidelity,
                    "composition_status": "pass" if record_passed else "fail",
                    "interface_validation": {
                        "status": "pass" if not interface_findings else "fail",
                        "findings": list(interface_findings),
                    },
                    "lowering_status": lowering.status,
                    "lowering_execution_binding": lowering.execution_binding,
                    "preflight_status": preflight.status,
                    "preflight_translator_id": preflight.translator_id,
                }
            )
        except (OSError, VehicleCompositionError, ValueError, KeyError, TypeError) as error:
            all_passed = False
            records.append(
                {
                    "path": str(path),
                    "composition_status": "fail",
                    "error": str(error),
                }
            )
    return {
        "schema": "taoryx.vehicle-composition-example-validation/v1alpha1",
        "directory": str(directory),
        "require_preflight": require_preflight,
        "status": "pass" if all_passed else "fail",
        "examples": records,
        "claim_boundary": (
            "This report compiles composition requests, validates their declared interface, and reports lowering "
            "and preflight readiness. With --require-preflight, an example must also bind a generic adapter or its "
            "exact declared batch factory. It does not execute a plant, evaluate truth objectives, or qualify a family."
        ),
    }
    ####


def _write_vehicle_interface_artifact(
    output_dir: Path,
    contract: object,
) -> dict[str, object]:
    """Write the exact control/status schema used by a public composition run."""

    from taoryx.vehicle_interface import VehicleInterfaceContract

    if not isinstance(contract, VehicleInterfaceContract):
        raise TypeError("vehicle interface artifact requires a resolved VehicleInterfaceContract")
    findings = validate_vehicle_interface_contract(contract)
    if findings:
        raise ValueError("vehicle interface contract failed validation: " + "; ".join(findings))
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / "vehicle_interface.json"
    payload = {
        **contract.as_dict(),
        "validation": {"status": "pass", "findings": []},
    }
    destination.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return {
        "path": str(destination),
        "interface_id": contract.id,
        "fingerprint_sha256": contract.fingerprint,
    }
    ####


def _finalize_vehicle_batch_execution_payload(
    payload: Mapping[str, object],
    *,
    output_dir: Path,
    composition: CompiledVehicleComposition,
    binding: VehicleExecutionBinding,
    composition_path: Path,
    max_steps: int | None,
    catalog: ResolvedVehicleCompositionCatalog,
) -> dict[str, object]:
    """Attach common public-run artifacts after one family-owned executor returns.

    Family executors own the plant, truth evaluation, and detailed artifacts.
    The CLI owns two cross-family Mission Composition records: the resolved interface
    and the exact invocation that produced this packet.  Keeping this here
    prevents the same provenance boilerplate from drifting across every
    source-owned batch factory.
    """

    result = dict(payload)
    result["vehicle_interface_artifact"] = _write_vehicle_interface_artifact(
        output_dir,
        resolve_vehicle_composition_interface_contract(composition, catalog=catalog),
    )
    result["reproduction_artifact"] = _write_vehicle_batch_reproduction_artifact(
        output_dir,
        composition=composition,
        binding=binding,
        composition_path=composition_path,
        max_steps=max_steps,
    )
    result["run_manifest"] = _write_composition_run_manifest(
        output_dir,
        composition=composition,
        binding=binding,
        composition_path=composition_path,
        payload=result,
        max_steps=max_steps,
    )
    return result
    ####


def _write_composition_run_manifest(
    output_dir: Path,
    *,
    composition: CompiledVehicleComposition,
    binding: VehicleExecutionBinding,
    composition_path: Path,
    payload: Mapping[str, object],
    max_steps: int | None,
) -> dict[str, object]:
    """Write the common Simulation Runtime manifest for a composition-owned run."""

    write_composition_run_artifact(output_dir, composition)
    packet = read_vehicle_execution_packet(output_dir / "execution.json")
    request = packet.host_execution.request
    if request.factory_id != binding.factory_id:
        raise ValueError("canonical execution packet factory does not match the resolved batch binding")
    if request.execution_mode != binding.execution_mode:
        raise ValueError("canonical execution packet execution mode does not match the resolved batch binding")
    if request.composition_id != composition.id or request.composition_identity_sha256 != composition.identity_sha256:
        raise ValueError("canonical execution packet does not match the requested composition")
    status = SimulationRuntimeStatus.PASSED if packet.outcome.passed else SimulationRuntimeStatus.INCOMPLETE
    execution_limit_reason = payload.get("execution_limit_reason")
    termination_reason = (
        "mission_pass"
        if packet.outcome.scope == "mission" and packet.outcome.passed
        else "local_screen_pass"
        if packet.outcome.scope == "local_screen" and packet.outcome.passed
        else str(execution_limit_reason)
        if isinstance(execution_limit_reason, str) and execution_limit_reason
        else "mission_or_screen_not_passed"
    )
    artifacts = artifact_inventory(output_dir)
    execution_artifact = next((item for item in artifacts if item.path == "execution.json"), None)
    if execution_artifact is None:
        raise ValueError("canonical execution packet was not included in the runtime artifact inventory")
    manifest = build_simulation_runtime_run_manifest(
        scenario_id=composition.id,
        status=status,
        expected_disposition=SimulationRuntimeStatus.DEVELOPMENT,
        operation="composition_batch",
        fidelity=str(composition.fidelity),
        realization=composition.control_realization,
        source_inputs=(source_input_record(composition_path, role="compiled_composition"),),
        runtime=default_runtime_identity(),
        integration={
            "factory_id": binding.factory_id,
            "execution_mode": binding.execution_mode,
            "max_steps": max_steps,
            "execution_packet": {
                "schema": packet.schema_id,
                "packet_identity_sha256": packet.packet_identity_sha256,
                "artifact_sha256": execution_artifact.sha256,
                "outcome_scope": packet.outcome.scope,
                "outcome_disposition": packet.outcome.disposition,
            },
        },
        time={"requested_duration_s": None, "accepted_start_s": None, "accepted_end_s": None},
        termination={
            "completed": status is SimulationRuntimeStatus.PASSED,
            "reason": termination_reason,
        },
        artifacts=artifacts,
        claim_boundary=(
            "This manifest identifies one composition-owned execution packet. It does not prove numerical accuracy, "
            "physical-effector behavior, vehicle qualification, or historical fidelity."
        ),
        reproduction_command=("taoryx", "vehicle", "run", str(composition_path), "--output-dir", str(output_dir)),
    )
    path = manifest.write_json(output_dir / "run-manifest.json")
    return {"path": str(path), "run_identity": manifest.run_identity, "status": manifest.status.value}
    ####


def _write_vehicle_batch_reproduction_artifact(
    output_dir: Path,
    *,
    composition: CompiledVehicleComposition,
    binding: VehicleExecutionBinding,
    composition_path: Path,
    max_steps: int | None,
) -> dict[str, object]:
    """Write an identity-bound public batch command without inventing a run.

    The source and output paths are intentionally recorded exactly as supplied
    to the CLI.  A copied packet may later need those files restored, but its
    provenance remains inspectable rather than being replaced by a generic
    example command.
    """

    command = [
        "taoryx",
        "vehicle",
        "run",
        str(composition_path),
        "--output-dir",
        str(output_dir),
    ]
    if max_steps is not None:
        command.extend(("--max-steps", str(max_steps)))
    destination = output_dir / "reproduction.txt"
    destination.write_text(
        "\n".join(
            (
                "# TAORYX Mission Composition public batch reproduction record",
                f"# composition_id: {composition.id}",
                f"# composition_identity_sha256: {composition.identity_sha256}",
                f"# execution_factory_id: {binding.factory_id}",
                f"# execution_mode: {binding.execution_mode}",
                shlex.join(command),
                "",
            )
        ),
        encoding="utf-8",
    )
    return {
        "path": str(destination),
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "factory_id": binding.factory_id,
        "execution_mode": binding.execution_mode,
    }
    ####


def _resolve_cli_case(path: Path, catalog_path: Path) -> ResolvedCase:
    """Load and resolve one case intent for CLI operations."""

    return resolve_case(load_case_intent(path), _trajectory_catalog(catalog_path))
    ####


def _case_command(arguments: argparse.Namespace) -> int:
    """Handle Alpha 2 case validation, resolution, explanation, and diff."""

    try:
        if arguments.case_command == "validate":
            resolved = _resolve_cli_case(arguments.path, arguments.catalog)
            print(f"valid: {resolved.case_id} ({resolved.identity_sha256})")
        elif arguments.case_command == "resolve":
            resolved = _resolve_cli_case(arguments.path, arguments.catalog)
            resolved.write_json(str(arguments.output))
            print(f"resolved: {resolved.case_id} ({resolved.identity_sha256})")
        elif arguments.case_command == "explain":
            resolved = _resolve_cli_case(arguments.path, arguments.catalog)
            _print_json(resolved.explain(arguments.parameter))
        else:
            left = _resolve_cli_case(arguments.left, arguments.catalog)
            right = _resolve_cli_case(arguments.right, arguments.catalog)
            _print_json(diff_resolved_cases(left, right))
        return 0
    except (OSError, KeyError, ResolutionError, TypeError, ValueError) as error:
        print(f"error: case-failed: {error}")
        return 2
    ####


def _schema_command(arguments: argparse.Namespace) -> int:
    """Handle Alpha 2 schema export."""

    try:
        resolved = _resolve_cli_case(arguments.path, arguments.catalog)
        payload: object
        if arguments.kind == "parameters":
            payload = {key: value.model_dump(mode="json") for key, value in resolved.parameters.items()}
        elif arguments.kind == "controls":
            payload = [item.model_dump(mode="json") for item in resolved.controls]
        else:
            payload = [item.model_dump(mode="json") for item in resolved.observations]
        _print_json(payload, arguments.output)
        return 0
    except (OSError, KeyError, ResolutionError, TypeError, ValueError) as error:
        print(f"error: schema-failed: {error}")
        return 2
    ####


def _render_artifact(arguments: argparse.Namespace) -> int:
    """Render one normalized artifact without loading source files."""

    try:
        artifact = RunArtifact.model_validate_json(arguments.path.read_text(encoding="utf-8"))
        if arguments.artifact_command == "validate":
            validation_payload: dict[str, object] = {
                "schema": "taoryx.run-artifact/v1",
                "path": str(arguments.path),
                "status": "passed",
                "scenario_identity": artifact.scenario_identity,
                "vehicle_count": len(artifact.vehicles),
                "claim_boundary": (
                    "Schema validation proves the normalized artifact is structurally readable. It does not prove "
                    "numerical accuracy, physical-effector behavior, or vehicle qualification."
                ),
            }
            if arguments.json:
                print(json.dumps(validation_payload, indent=2, sort_keys=True))
            else:
                print(f"valid artifact: {arguments.path}")
            return 0
        if arguments.artifact_command == "inspect":
            inspection_payload = _artifact_inspection_payload(artifact)
            if arguments.json:
                print(json.dumps(inspection_payload, indent=2, sort_keys=True))
            else:
                print(f"problem: {artifact.problem}")
                print(f"schema_version: {artifact.schema_version}")
                for vehicle in inspection_payload["vehicles"]:
                    print(
                        "vehicle: "
                        f"{vehicle['vehicle_id']} dynamics={vehicle['dynamics']} "
                        f"samples={vehicle['sample_count']} "
                        f"time={vehicle['time_start_s']}..{vehicle['time_end_s']} s"
                    )
                    print("  channels: " + ", ".join(vehicle["channels"]))
                print(f"events: {inspection_payload['event_count']}")
                print(f"commands: {inspection_payload['command_count']}")
            return 0
        if arguments.artifact_command == "html":
            render_run_artifact_html(
                artifact,
                arguments.output,
                vehicle_id=arguments.vehicle,
                channels=tuple(arguments.channel),
            )
            print(f"rendered artifact HTML: {arguments.output}")
        else:
            plots = render_run_artifact_plots(
                artifact,
                arguments.output_dir,
                vehicle_id=arguments.vehicle,
                channels=tuple(arguments.channel),
            )
            print(f"rendered {len(plots)} artifact plot(s): {arguments.output_dir}")
    except (OSError, TypeError, ValueError) as error:
        print(f"error: artifact-render-failed: {error}")
        return 2
    return 0


def _artifact_inspection_payload(artifact: RunArtifact) -> _ArtifactInspectionPayload:
    """Return a compact diagnostic projection of one normalized artifact."""

    vehicles: list[_ArtifactInspectionVehicle] = [
        {
            "vehicle_id": vehicle_id,
            "name": vehicle.name,
            "kind": vehicle.kind.value,
            "dynamics": vehicle.dynamics.value,
            "sample_count": len(vehicle.times),
            "time_start_s": vehicle.times[0] if vehicle.times else None,
            "time_end_s": vehicle.times[-1] if vehicle.times else None,
            "channels": sorted(vehicle.channels),
            "segments": [segment.title for segment in vehicle.segments],
        }
        for vehicle_id, vehicle in sorted(artifact.vehicles.items())
    ]
    return {
        "schema": "taoryx.runtime-artifact-inspection/v1alpha1",
        "artifact_schema_version": artifact.schema_version,
        "problem": artifact.problem,
        "scenario_identity": artifact.scenario_identity,
        "vehicles": vehicles,
        "event_count": len(artifact.events),
        "command_count": len(artifact.commands),
        "termination": artifact.termination.as_dict(),
        "claim_boundary": (
            "This is an artifact-shape and telemetry-presence inspection. It does not establish model validity, "
            "controller quality, physical-effector behavior, or vehicle qualification."
        ),
    }
    ####


def _inspect_table(arguments: argparse.Namespace) -> int:
    """Handle the renderer-independent table inspection command."""

    try:
        artifact = inspect_table_file(arguments.path)
        selected = artifact.table(arguments.table_name)
        query = _parse_query(arguments.at)
        probe = explain_interpolation(selected, query) if query else None
        payload = artifact.to_dict()
        payload["selected_table"] = selected.to_dict()
        if probe is not None:
            payload["probe"] = probe.to_dict()
        if arguments.html:
            _write_table_html(arguments.html, payload, selected, probe)
        if arguments.json:
            print(json.dumps(payload, indent=2, default=str))
        elif not arguments.html:
            print(artifact.format_catalog())
            if probe is not None:
                print(json.dumps(probe.to_dict(), indent=2))
        return 0
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"error: table-inspect-failed: {error}")
        return 2


####


def _parse_query(values: list[str]) -> dict[str, float]:
    query: dict[str, float] = {}
    for item in values:
        name, separator, raw_value = item.partition("=")
        if not separator or not name:
            raise ValueError(f"invalid query {item!r}; expected AXIS=VALUE")
        query[name.casefold()] = float(raw_value)
    return query


####


def _write_table_html(path: Path, payload: dict[str, object], table: TableInspection, probe: InterpolationExplanation | None) -> None:
    """Write a standalone inspection page with an optional prepared-table PNG."""

    path.parent.mkdir(parents=True, exist_ok=True)
    image = ""
    prepared = table.prepared
    if prepared is not None:
        from taoryx.visualization import render_table_png

        image_data = base64.b64encode(
            render_table_png(
                prepared,
                axis_labels=table.independent_variables,
                value_label=table.table_type,
                title=table.name,
            )
        ).decode("ascii")
        image = f'<img alt="table plot" src="data:image/png;base64,{image_data}">'
    probe_json = "" if probe is None else f"<h2>Probe</h2><pre>{escape(json.dumps(probe.to_dict(), indent=2))}</pre>"
    body = escape(json.dumps(payload, indent=2, default=str))
    path.write_text(
        f"<html><head><meta charset='utf-8'><title>TAORYX Table Explorer</title></head><body><h1>TAORYX Table Explorer</h1><pre>{body}</pre>{probe_json}{image}</body></html>\n",
        encoding="utf-8",
    )


####
####


if __name__ == "__main__":
    raise SystemExit(main())
####
