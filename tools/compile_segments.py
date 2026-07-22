"""Compile external declarative segment catalogs into native problem files."""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.segmentation import SegmentationCompileError, compile_catalog, lint_catalog

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("lint", "build", "run"), default="build")
    parser.add_argument("--catalog", type=Path, default=ROOT / "verification/segmentation_catalog.yaml")
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--table", type=Path, action="append", default=[])
    parser.add_argument("--output-dir", type=Path, default=ROOT / "artifacts/segmentation/runs")
    parser.add_argument("--max-steps", type=int, default=100000)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--profile", choices=tuple(profile.value for profile in GrammarProfile), default=GrammarProfile.TAORYX.value)
    args = parser.parse_args()
    try:
        catalog = lint_catalog(args.catalog, args.root)
        if args.command == "lint":
            print(f"lint passed: {len(catalog.scenarios)} scenario(s)")
            return 0
        outputs = compile_catalog(args.catalog, args.root)
        for problem, manifest, audit in outputs:
            print(f"compiled {problem.relative_to(args.root)}")
            print(f"  manifest {manifest.relative_to(args.root)}")
            print(f"  audit {audit.relative_to(args.root)}")
        if args.command == "run":
            if args.max_steps <= 0:
                raise SegmentationCompileError("--max-steps must be positive")
            reports: list[dict[str, object]] = []
            exit_codes: list[int] = []
            for scenario, (problem, _, _) in zip(catalog.scenarios, outputs, strict=True):
                table_paths = tuple(args.root / path for path in scenario.runtime_tables) + tuple(args.table)
                report = run_files(
                    problem,
                    table_paths,
                    output_dir=args.output_dir / scenario.id,
                    max_steps=args.max_steps,
                    profile=args.profile,
                )
                report_dict = report.as_dict()
                artifact_paths: list[str] = []
                for index, artifact in enumerate(report.artifacts, start=1):
                    artifact_path = args.output_dir / scenario.id / f"run-artifact-{index}.json"
                    artifact.write_json(artifact_path)
                    artifact_paths.append(str(artifact_path))
                report_dict["segmentation_artifacts"] = artifact_paths
                reports.append(report_dict)
                exit_codes.append(report.exit_code)
            if args.json:
                print(json.dumps({"scenarios": reports}, indent=2, sort_keys=True))
            else:
                for item in reports:
                    raw_diagnostics = item.get("diagnostics", [])
                    diagnostics = raw_diagnostics if isinstance(raw_diagnostics, list) else []
                    codes = ", ".join(
                        str(diagnostic.get("code"))
                        for diagnostic in diagnostics[:4]
                        if isinstance(diagnostic, Mapping)
                    ) if diagnostics else "none"
                    print(f"run {item['problem']}: exit={item['exit_code']} cases={item['cases']} diagnostics={codes}")
            args.output_dir.mkdir(parents=True, exist_ok=True)
            compact_reports = [
                {
                    key: item[key]
                for key in ("problem", "tables", "cases", "results", "diagnostics", "outputs", "segmentation_artifacts", "exit_code")
                    if key in item
                }
                for item in reports
            ]
            (args.output_dir / "segmentation-run-summary.json").write_text(
                json.dumps({"schema_version": 1, "scenarios": compact_reports}, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            return 0 if all(code == 0 for code in exit_codes) else 1
    except (OSError, SegmentationCompileError, ValueError) as error:
        print(f"error: segmentation-{args.command}-failed: {error}")
        return 2
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
