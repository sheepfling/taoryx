"""Parse every indexed example and execute the Taoryx full-example corpus."""

from __future__ import annotations

import argparse
import importlib
import json
import sys
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.ingest import ingest_file
from taoryx.outputs import RunArtifact
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _source_paths(family: str) -> tuple[Path, ...]:
    families = ("taos96", "taoryx") if family == "all" else (family,)
    roots = tuple(
        root
        for selected in families
        for root in (
            ROOT / "examples" / "chapter03" if selected == "taos96" else ROOT / "examples" / selected / "syntax_fragments",
            ROOT / "examples" / "chapter04" if selected == "taos96" else ROOT / "examples" / selected / "full_examples",
        )
    )
    return tuple(sorted(path for root in roots for path in root.rglob("*") if path.suffix.casefold() in {".prb", ".tbl"}))


def _parse_report(family: str) -> tuple[list[dict[str, object]], int]:
    reports: list[dict[str, object]] = []
    errors = 0
    for path in _source_paths(family):
        selected_family = "taos96" if any(
            path.is_relative_to(ROOT / "examples" / root)
            for root in ("chapter03", "chapter04")
        ) else "taoryx"
        profile = GrammarProfile(selected_family)
        ingested = ingest_file(path, profile=profile)
        diagnostics = [item.model_dump(mode="json") for item in ingested.diagnostics]
        errors += sum(item["severity"] == "error" for item in diagnostics)
        reports.append(
            {
                "path": str(path.relative_to(ROOT)),
                "profile": selected_family,
                "kind": ingested.kind.value,
                "valid": not any(item["severity"] == "error" for item in diagnostics),
                "diagnostics": diagnostics,
                "semantic_ast": ingested.document.model_dump(mode="json"),
            }
        )
    return reports, errors


def _execute_taoryx(destination: Path, max_steps: int) -> list[dict[str, object]]:
    reports: list[dict[str, object]] = []
    source_root = ROOT / "examples" / "taoryx" / "full_examples"
    for problem in sorted(source_root.glob("*.prb")):
        case_root = destination / problem.stem
        report = run_files(problem, output_dir=case_root, max_steps=max_steps, profile=GrammarProfile.TAORYX)
        artifacts = []
        for index, artifact in enumerate(report.artifacts, start=1):
            artifact.write_json(case_root / f"case-{index}-artifact.json")
            artifact.write_csv(case_root / f"case-{index}-telemetry.csv")
            render_run_artifact_plots(artifact, case_root / "plots" / f"case-{index}", dpi=100)
            artifacts.append(f"case-{index}-artifact.json")
        payload = report.as_dict()
        payload["artifact_files"] = artifacts
        (case_root / "run-report.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        reports.append(payload)
    return reports


def _execute_taos96(destination: Path, max_steps: int) -> list[dict[str, object]]:
    """Execute TAOS96-compatible examples with every bundled table deck."""

    reports: list[dict[str, object]] = []
    source_root = ROOT / "examples" / "chapter04"
    tables = tuple(path for path in _source_paths("taos96") if path.suffix.casefold() == ".tbl")
    for problem in sorted(source_root.glob("*.prb")):
        case_root = destination / problem.stem
        report = run_files(problem, tables, output_dir=case_root, max_steps=max_steps, profile=GrammarProfile.TAOS96)
        artifacts = []
        for index, artifact in enumerate(report.artifacts, start=1):
            artifact.write_json(case_root / f"case-{index}-artifact.json")
            artifact.write_csv(case_root / f"case-{index}-telemetry.csv")
            render_run_artifact_plots(artifact, case_root / "plots" / f"case-{index}", dpi=100)
            artifacts.append(f"case-{index}-artifact.json")
        payload = report.as_dict()
        payload["artifact_files"] = artifacts
        (case_root / "run-report.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        reports.append(payload)
    return reports


def _execute_showcases(destination: Path) -> list[dict[str, object]]:
    """Regenerate the four polished runtime showcases in the family index."""

    showcase_ids = (
        "suborbital_ballistic_return",
        "orbital_insertion_coast_reentry",
        "quadcopter_drone_racetrack",
        "california_to_hawaii",
    )
    reports: list[dict[str, object]] = []
    for showcase_id in showcase_ids:
        root = destination / showcase_id
        module = importlib.import_module(f"examples.showcases.{showcase_id}.run_showcase")
        module.generate(root)
        source = root / "run.json"
        if not source.exists():
            source = root / "telemetry.json"
        artifact = RunArtifact.model_validate_json(source.read_text(encoding="utf-8"))
        artifact.write_json(root / "artifact.json")
        artifact.write_csv(root / "telemetry.csv")
        render_run_artifact_plots(artifact, root / "plots", dpi=100)
        reports.append({"id": showcase_id, "artifact": str(root / "artifact.json")})
    return reports


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", choices=("all", "taos96", "taoryx"), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="execute Taoryx full examples after parsing")
    parser.add_argument("--max-steps", type=int, default=20)
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    parser_reports, parse_errors = _parse_report(arguments.family)
    (arguments.output / "parser-report.json").write_text(json.dumps(parser_reports, indent=2) + "\n", encoding="utf-8")
    taos96_reports = _execute_taos96(arguments.output / "runs" / "taos96", arguments.max_steps) if arguments.execute and arguments.family in {"all", "taos96"} else []
    run_reports = _execute_taoryx(arguments.output / "runs" / "taoryx", arguments.max_steps) if arguments.execute and arguments.family in {"all", "taoryx"} else []
    showcase_reports = _execute_showcases(arguments.output / "showcases") if arguments.execute and arguments.family in {"all", "taoryx"} else []
    runtime_reports = (*taos96_reports, *run_reports)
    summary = {
        "family": arguments.family,
        "source_files": len(parser_reports),
        "parse_errors": parse_errors,
        "executed_cases": sum(item["cases"] for item in run_reports),
        "taos96_executed_cases": sum(item["cases"] for item in taos96_reports),
        "runtime_failures": sum(item["exit_code"] == 2 and not any(diagnostic["code"] == "missing-runtime-table" for diagnostic in item["diagnostics"]) for item in runtime_reports),
        "runtime_unavailable": sum(
            any(diagnostic["code"] == "missing-runtime-table" for diagnostic in item["diagnostics"])
            for item in taos96_reports
        ),
        "runtime_incomplete": sum(item["exit_code"] == 1 for item in runtime_reports),
        "taos96_reports": taos96_reports,
        "run_reports": run_reports,
        "showcases": showcase_reports,
    }
    (arguments.output / "corpus-report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key != "run_reports"}, indent=2))
    return 1 if parse_errors or summary["runtime_failures"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
