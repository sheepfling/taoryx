"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.integration import available_integrators

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
    run.add_argument("--integrator", choices=tuple(item.value for item in available_integrators()))
    optimizers = subparsers.add_parser("optimizers", help="inspect available numerical backends")
    optimizers_subparsers = optimizers.add_subparsers(dest="optimizer_command", required=True)
    optimizers_subparsers.add_parser("list", help="list installed optimization backends")
    integrators = subparsers.add_parser("integrators", help="inspect available integration backends")
    integrators_subparsers = integrators.add_subparsers(dest="integrator_command", required=True)
    integrators_subparsers.add_parser("list", help="list installed integration backends")
    arguments = parser.parse_args(argv)
    if arguments.command == "optimizers":
        for backend in available_optimizers():
            print(backend.value)
        return 0
    if arguments.command == "integrators":
        for integrator_name in available_integrators():
            print(integrator_name.value)
        return 0
    if arguments.max_steps <= 0:
        parser.error("--max-steps must be positive")
    report = run_files(
        arguments.problem,
        tuple(arguments.tables),
        output_dir=arguments.output_dir,
        max_steps=arguments.max_steps,
        integrator=arguments.integrator,
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
####


if __name__ == "__main__":
    raise SystemExit(main())
####
