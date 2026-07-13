"""Command-line interface for the executable TAOS runtime path."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

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
    arguments = parser.parse_args(argv)
    report = run_files(arguments.problem, tuple(arguments.tables), output_dir=arguments.output_dir, max_steps=arguments.max_steps)
    if arguments.report:
        arguments.report.write_text(json.dumps(report.as_dict(), indent=2) + "\n", encoding="utf-8")
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
