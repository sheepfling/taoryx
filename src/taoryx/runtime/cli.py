"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import base64
import json
from html import escape
from pathlib import Path

from taoryx.integration import available_integrator_descriptions, available_integrators
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.outputs import RunArtifact
from taoryx.scenario import ScenarioCompileError, ScenarioCompiler
from taoryx.table_explorer import InterpolationExplanation, TableInspection, explain_interpolation, inspect_table_file
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots

from .optimization_runtime import available_optimizers
from .runner import run_files


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
