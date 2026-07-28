"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import base64
import json
import math
from html import escape
from pathlib import Path

from taoryx.integration import available_integrator_descriptions, available_integrators
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.outputs import RunArtifact
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
from taoryx.trajectory.resolution import ResolutionError
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots
from taoryx.x15_native_replay import write_x15_native_boundary_replay
from taoryx.x15_reachability import write_x15_reachability_bundle

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
    run.add_argument(
        "--integrator",
        choices=tuple(item.value for item in available_integrators()),
        help=(
            "integration backend: euler for fast tests, rk4 for fixed-step runs, "
            "or scipy-* if installed"
        ),
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
    reachability_x15_native = reachability_subparsers.add_parser("x15-native-replay", help="replay selected X-15 envelope points through native rigid-body cases")
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
        roundtrip = json.loads(
            Path("families/a320_openap_jsbsim_pseudo6dof/validation/roundtrip-report.json").read_text(encoding="utf-8")
        )
        runtime_qualification = json.loads(
            Path("families/a320_openap_jsbsim_pseudo6dof/validation/runtime-qualification.json").read_text(encoding="utf-8")
        )
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
                _print_json(
                    {
                        name: {"meaning": semantic.meaning}
                        for name, semantic in sorted(catalog.study_semantics.items())
                    }
                )
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
