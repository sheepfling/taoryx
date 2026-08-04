"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import base64
import json
import math
import shlex
from collections.abc import Mapping
from html import escape
from pathlib import Path

from taoryx.batch_episode_parity_dispatch import verify_serialized_declared_batch_episode_parity
from taoryx.composition_episode import open_vehicle_composition_episode
from taoryx.composition_policy import replay_composition_policy_trace_file
from taoryx.composition_result_catalog import index_composition_results, write_composition_release_catalog
from taoryx.hl20_source_release_composition_execution import execute_hl20_source_booster_release_composition
from taoryx.hummingbird_composition_execution import execute_hummingbird_pseudo_composition
from taoryx.integration import available_integrator_descriptions, available_integrators
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language_backed_execution import execute_powered_fixed_wing_composition
from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition
from taoryx.local_direct_wrench_composition_execution import execute_local_direct_wrench_composition
from taoryx.nesc_composition_execution import execute_nesc_source_replay_composition
from taoryx.outputs import RunArtifact
from taoryx.passive_tumbling_composition_execution import execute_passive_tumbling_composition
from taoryx.product_three_maturity import build_product_three_maturity_report
from taoryx.reachability_catalog import ReachabilityCatalog, load_reachability_catalog
from taoryx.reachability_envelope import (
    ReachabilityFidelity,
    RocketGlideVehicle,
    TerminalCriteria,
    generate_launch_grid,
    rerun_timed_out_artifact,
    run_reachability_envelope,
)
from taoryx.reachability_visualization import load_reachability_artifact, render_reachability_plot_bundle
from taoryx.reduced_fixed_wing_execution import execute_reduced_fixed_wing_composition
from taoryx.scenario import ScenarioCompileError, ScenarioCompiler
from taoryx.table_explorer import InterpolationExplanation, TableInspection, explain_interpolation, inspect_table_file
from taoryx.trajectory import (
    A320OpenAPModel,
    A320OpenAPOperatingPoint,
    A320Pseudo6DOFModel,
    A320Pseudo6DOFOperatingPoint,
    FamilyCatalog,
    ResolvedCase,
    diff_resolved_cases,
    load_case_intent,
    load_daveml_family_graph,
    load_daveml_family_import,
    load_family_catalog,
    resolve_case,
)
from taoryx.trajectory.evaluation import TrajectoryEvaluation
from taoryx.trajectory.resolution import ResolutionError
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
from taoryx.vehicle_execution_bindings import VehicleExecutionBinding, resolve_vehicle_execution_binding
from taoryx.vehicle_execution_preflight import build_semantic_preflight_handler_report, preflight_vehicle_composition
from taoryx.vehicle_integration_intake import (
    ExistingFamilyIntakeRequest,
    NewTopologyIntakeRequest,
    StrategySelectionError,
    build_existing_family_intake_blueprint,
    build_new_topology_intake_scaffold,
)
from taoryx.vehicle_integration_pipeline import (
    validate_all_vehicle_integration_pipelines,
    validate_vehicle_integration_pipeline,
    write_vehicle_integration_packet,
)
from taoryx.vehicle_integration_readiness import (
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)
from taoryx.vehicle_interface import build_vehicle_interface_catalog_report, resolve_vehicle_interface_contract, validate_vehicle_interface_contract
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots
from taoryx.x15_native_replay import write_x15_native_boundary_replay
from taoryx.x15_reachability import write_x15_reachability_bundle
from taoryx.x15_staged_composition_execution import execute_x15_staged_reachability_composition

from .optimization_runtime import available_optimizers
from .runner import run_files


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


def _criteria_from_arguments(arguments: argparse.Namespace) -> TerminalCriteria | None:
    names = (
        "min_speed_m_s",
        "max_speed_m_s",
        "target_x_m",
        "target_y_m",
        "target_z_m",
        "max_impact_radius_m",
        "min_impact_speed_m_s",
        "max_impact_speed_m_s",
    )
    if not arguments.require_ground_contact and not any(getattr(arguments, name) is not None for name in names):
        return None
    return TerminalCriteria(
        min_speed_m_s=0.0 if arguments.min_speed_m_s is None else arguments.min_speed_m_s,
        max_speed_m_s=math.inf if arguments.max_speed_m_s is None else arguments.max_speed_m_s,
        require_ground_contact=arguments.require_ground_contact,
        target_position_m=(
            0.0 if arguments.target_x_m is None else arguments.target_x_m,
            0.0 if arguments.target_y_m is None else arguments.target_y_m,
            0.0 if arguments.target_z_m is None else arguments.target_z_m,
        ),
        max_impact_radius_m=arguments.max_impact_radius_m,
        min_impact_speed_m_s=arguments.min_impact_speed_m_s,
        max_impact_speed_m_s=arguments.max_impact_speed_m_s,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="taoryx", description="Typed TAOS parser and runtime")
    subparsers = parser.add_subparsers(dest="command", required=True)
    run = subparsers.add_parser("run", help="execute a .prb problem with optional .tbl files")
    run.add_argument("problem", type=Path)
    run.add_argument("tables", type=Path, nargs="*")
    run.add_argument("--output-dir", type=Path, default=Path("."))
    run.add_argument("--report", type=Path)
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
    scenario = subparsers.add_parser("scenario", help="compile and inspect resolved scenario caches")
    scenario_subparsers = scenario.add_subparsers(dest="scenario_command", required=True)
    compile_scenario = scenario_subparsers.add_parser("compile", help="validate source and write a resolved scenario cache")
    compile_scenario.add_argument("problem", type=Path)
    compile_scenario.add_argument("tables", type=Path, nargs="*")
    compile_scenario.add_argument("--output", type=Path, required=True)
    compile_scenario.add_argument("--profile", choices=tuple(profile.value for profile in GrammarProfile), default=GrammarProfile.TAOS96.value)
    compile_scenario.add_argument("--seed", type=int)
    compile_scenario.add_argument("--integrator")
    compile_scenario.add_argument("--json", action="store_true")
    artifact = subparsers.add_parser("artifact", help="inspect normalized run artifacts")
    artifact_subparsers = artifact.add_subparsers(dest="artifact_command", required=True)
    artifact_html = artifact_subparsers.add_parser("html", help="render a run artifact as standalone HTML")
    artifact_html.add_argument("path", type=Path)
    artifact_html.add_argument("--output", type=Path, required=True)
    artifact_html.add_argument("--vehicle")
    artifact_html.add_argument("--channel", action="append", default=[])
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
    reachability = subparsers.add_parser("reachability", help="inspect Alpha 3 reachability catalogs")
    reachability_subparsers = reachability.add_subparsers(dest="reachability_command", required=True)
    reachability_list = reachability_subparsers.add_parser("list", help="list reachability catalog entries")
    reachability_list.add_argument("kind", choices=("profiles", "families", "semantics"))
    reachability_list.add_argument("--catalog", type=Path, default=Path("verification/reachability_profile_catalog.yaml"))
    reachability_inspect = reachability_subparsers.add_parser("inspect", help="inspect one reachability catalog entry")
    reachability_inspect.add_argument("kind", choices=("profile", "family", "semantic"))
    reachability_inspect.add_argument("identifier")
    reachability_inspect.add_argument("--catalog", type=Path, default=Path("verification/reachability_profile_catalog.yaml"))
    reachability_run = reachability_subparsers.add_parser("run", help="run the reduced-order rocket/glide envelope fixture")
    reachability_run.add_argument("--fidelity", choices=tuple(item.value for item in ReachabilityFidelity), default=ReachabilityFidelity.POINT_MASS_3DOF.value)
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
        help="join Product 3 topology, authoring, execution, and parity coverage without evidence promotion",
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
        help="show the Product 3 composition authoring worklist for one vehicle family",
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
        help="aggregate Product 3 authoring worklists without collapsing family-specific gaps",
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
        help="read and validate one normalized Product 3 evaluation artifact",
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
        return _compile_scenario(arguments)
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
    if arguments.report:
        try:
            arguments.report.parent.mkdir(parents=True, exist_ok=True)
            arguments.report.write_text(json.dumps(report.as_dict(), indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            print(f"error: report-write-failed: {error}")
            return 2
    if arguments.json:
        print(json.dumps(report.as_dict(), indent=2))
    else:
        for diagnostic in report.diagnostics:
            location = diagnostic.location
            prefix = f"{location.path}:{location.line}: " if location else ""
            print(f"{diagnostic.severity}: {prefix}{diagnostic.code}: {diagnostic.message}")
        print(f"executed {report.cases} case(s), exit={report.exit_code}")
        for output in report.outputs:
            print(f"output: {output}")
    return report.exit_code


def _daveml_command(arguments: argparse.Namespace) -> int:
    """Run fail-closed DAVE-ML family smoke verification."""

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


def _reachability_catalog(path: Path) -> ReachabilityCatalog:
    """Load the configured Alpha 3 reachability catalog for CLI commands."""

    return load_reachability_catalog(path)
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
    """Handle Alpha 3 reachability catalog inspection."""

    try:
        if arguments.reachability_command == "x15":
            reachability_bundle = write_x15_reachability_bundle(
                arguments.output_dir,
                workers=arguments.workers,
                step_size_s=arguments.step_size_s,
                horizon_s=arguments.horizon_s,
                criteria=_criteria_from_arguments(arguments),
                dpi=arguments.dpi,
            )
            print(f"wrote X-15 reachability bundle: {reachability_bundle.manifest_path.parent}")
            return 0
        if arguments.reachability_command == "x15-native-replay":
            native_bundle = write_x15_native_boundary_replay(
                arguments.envelope,
                arguments.output_dir,
                max_points=arguments.max_points,
                duration_s=arguments.duration_s,
                max_steps=arguments.max_steps,
            )
            print(f"wrote X-15 native replay bundle: {native_bundle.manifest_path.parent}")
            return 0
        if arguments.reachability_command == "run":
            azimuths = arguments.azimuth_deg or (-30.0, 0.0, 30.0)
            elevations = arguments.elevation_deg or (35.0, 50.0, 65.0)
            banks = arguments.bank_deg or (-30.0, 0.0, 30.0)
            commands = generate_launch_grid(
                tuple(math.radians(value) for value in azimuths),
                tuple(math.radians(value) for value in elevations),
                tuple(math.radians(value) for value in banks),
            )
            result = run_reachability_envelope(
                RocketGlideVehicle(),
                commands,
                fidelity=ReachabilityFidelity(arguments.fidelity),
                step_size_s=arguments.step_size_s,
                horizon_s=arguments.horizon_s,
                workers=arguments.workers,
                criteria=_criteria_from_arguments(arguments),
            )
            _print_json(
                result.as_dict(include_trajectories=arguments.include_trajectories or not arguments.omit_trajectories),
                arguments.output,
            )
            return 0
        if arguments.reachability_command == "rerun-timeouts":
            payload = load_reachability_artifact(arguments.envelope)
            result = rerun_timed_out_artifact(
                payload,
                horizon_s=arguments.horizon_s,
                step_size_s=arguments.step_size_s,
                workers=arguments.workers,
            )
            result.write_json(arguments.output)
            print(f"wrote timeout rerun: {arguments.output}")
            return 0
        if arguments.reachability_command == "plot":
            sources = tuple(load_reachability_artifact(path) for path in arguments.compare)
            report = render_reachability_plot_bundle(
                load_reachability_artifact(arguments.path),
                arguments.output_dir,
                comparison_sources=sources,
                dpi=arguments.dpi,
            )
            print(f"rendered {len(report.plot_paths)} reachability plot(s): {arguments.output_dir}")
            return 0
        catalog = _reachability_catalog(arguments.catalog)
        if arguments.reachability_command == "list":
            if arguments.kind == "profiles":
                _print_json(
                    [
                        {
                            "profile_id": profile.id,
                            "archetype": profile.archetype,
                            "status": profile.status,
                            "configurations": list(profile.configurations),
                            "coordinates": list(profile.coordinates),
                            "products": list(profile.products),
                        }
                        for profile in catalog.profiles
                    ]
                )
            elif arguments.kind == "families":
                _print_json(
                    [
                        {
                            "family_id": family.id,
                            "status": family.status,
                            "configurations": list(family.configurations),
                            "profiles": list(family.profiles),
                        }
                        for family in catalog.vehicle_families
                    ]
                )
            else:
                _print_json({name: {"meaning": semantic.meaning} for name, semantic in sorted(catalog.study_semantics.items())})
        elif arguments.kind == "profile":
            _print_json(catalog.profile(arguments.identifier).model_dump(mode="json"))
        elif arguments.kind == "family":
            _print_json(catalog.family(arguments.identifier).model_dump(mode="json"))
        else:
            _print_json(catalog.semantic(arguments.identifier).model_dump(mode="json"))
        return 0
    except (OSError, KeyError, TypeError, ValueError) as error:
        print(f"error: reachability-failed: {error}")
        return 2
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
                            "lowering_status": lowering.status,
                            "execution_binding": lowering.execution_binding,
                        },
                        "claim_boundary": (
                            "Mission validation proves only immutable semantic compilation and interface conformance. "
                            "A non-ready adapter, missing batch factory, or failed mission execution remains visible "
                            "and cannot be promoted by this command."
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
            binding = resolve_vehicle_execution_binding(composition, "batch")
            if binding.factory_id == "language_backed_powered_fixed_wing.v1":
                language_execution = execute_powered_fixed_wing_composition(
                    composition,
                    arguments.output_dir,
                    max_steps=arguments.max_steps,
                )
                payload = _finalize_vehicle_batch_execution_payload(
                    language_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if language_execution.mission_pass else 1
            if binding.factory_id in {"reduced_fixed_wing_openap.v1", "reduced_fixed_wing_f16_source.v1"}:
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for this reduced fixed-wing adapter yet")
                reduced_execution = execute_reduced_fixed_wing_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    reduced_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if reduced_execution.mission_pass else 1
            if binding.factory_id == "hummingbird_aggregate_thrust_pseudo_batch.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the Hummingbird pseudo batch adapter yet")
                hummingbird_execution = execute_hummingbird_pseudo_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    hummingbird_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if hummingbird_execution.mission_pass else 1
            if binding.factory_id == "nesc_source_replay.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the NESC source-replay adapter")
                nesc_execution = execute_nesc_source_replay_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    nesc_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if nesc_execution.mission_pass else 1
            if binding.factory_id == "hl20_source_booster_release_replay.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the HL-20 source-release replay adapter")
                hl20_execution = execute_hl20_source_booster_release_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    hl20_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if hl20_execution.mission_pass else 1
            if binding.factory_id == "x15_staged_reachability.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the X-15 staged reachability adapter")
                x15_execution = execute_x15_staged_reachability_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    x15_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if x15_execution.mission_pass else 1
            if binding.factory_id == "local_direct_wrench_screen.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the local direct-wrench screen adapter")
                local_screen_execution = execute_local_direct_wrench_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    local_screen_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if local_screen_execution.screen_pass else 1
            if binding.factory_id == "passive_tumbling_direct_release.v1":
                if arguments.max_steps is not None:
                    raise ValueError("--max-steps is not available for the passive tumbling batch adapter")
                tumbling_execution = execute_passive_tumbling_composition(composition, arguments.output_dir)
                payload = _finalize_vehicle_batch_execution_payload(
                    tumbling_execution.as_dict(),
                    output_dir=arguments.output_dir,
                    composition=composition,
                    binding=binding,
                    composition_path=arguments.composition,
                    max_steps=arguments.max_steps,
                    catalog=catalog,
                )
                _print_json(payload)
                return 0 if tumbling_execution.mission_pass else 1
            raise ValueError(f"batch execution factory is declared but not implemented: {binding.factory_id!r}")
        if arguments.vehicle_command == "replay-policy":
            composition = load_compiled_vehicle_composition(arguments.composition)
            replay_report = replay_composition_policy_trace_file(composition, arguments.trace)
            _print_json(replay_report.as_dict(), arguments.output)
            return 0
        if arguments.vehicle_command == "batch-episode-parity":
            composition = load_compiled_vehicle_composition(arguments.composition)
            trace_payload = json.loads(arguments.trace.read_text(encoding="utf-8"))
            if not isinstance(trace_payload, Mapping):
                raise ValueError("policy trace JSON must contain one object")
            parity_report = verify_serialized_declared_batch_episode_parity(composition, trace_payload)
            _print_json(parity_report.as_dict(), arguments.output)
            return 0 if parity_report.status == "pass" else 1
        if arguments.vehicle_command == "episode-info":
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
            report = build_product_three_maturity_report(
                catalog,
                check_execution_witnesses=arguments.check_execution_witnesses,
                execute_batch_witnesses=arguments.execute_batch_witnesses,
                execute_parity_witnesses=arguments.execute_parity_witnesses,
                results_directory=arguments.results_dir,
            )
            _print_json(report)
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
    """Export the existing intake generator through the Product 3 CLI.

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
    """Expose source-family onboarding gates through the Product 3 CLI.

    This command intentionally reuses the provider-neutral integration records
    rather than treating a composition-registry entry as proof that a source
    plant is trim-ready, runnable, or qualified.  It makes the missing work
    visible to a vehicle author at the same public surface used for intake and
    composition discovery.
    """

    try:
        if arguments.vehicle_integration_command == "readiness":
            reports = (
                validate_all_vehicle_integration_readiness()
                if arguments.family == "all"
                else (validate_vehicle_integration_readiness(arguments.family),)
            )
            payload = {
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
            validate_all_vehicle_integration_pipelines()
            if arguments.family == "all"
            else (validate_vehicle_integration_pipeline(arguments.family),)
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
                "Staged provider-neutral integration evidence only. It does not execute a mission, prove "
                "physical allocation, or qualify a vehicle family."
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
    The CLI owns two cross-family Product 3 records: the resolved interface
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
    return result
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
                "# TAORYX Product 3 public batch reproduction record",
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
