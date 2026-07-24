"""Execute and classify the four Alpha 1 manual-example families."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import tempfile
from pathlib import Path
from typing import Any

import yaml

from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]
FIXTURE_ROOT = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive"
CONFIG = ROOT / "verification/alpha1_manual_execution.yaml"
DEFAULT_OUTPUT = ROOT / "artifacts/verification/alpha1/manual_examples/report.json"


def _sha256(path: Path) -> str:
    """Return a source file hash for the execution record."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _status(report: Any) -> tuple[str, str | None]:
    """Classify runtime completion without hiding a known blocker."""

    errors = [item for item in report.diagnostics if item.severity.value == "error"]
    if report.exit_code == 0 and not errors:
        return "pass", None
    if errors:
        return "blocked", "; ".join(f"{item.code}: {item.message}" for item in errors)
    return "warning", "; ".join(item.message for item in report.diagnostics)
    ####


def _report_summary(report: Any) -> dict[str, Any]:
    """Project a runtime report into compact machine-readable evidence."""

    status, reason = _status(report)
    return {
        "status": status,
        "blocker": reason,
        "case_count": report.cases,
        "result_count": len(report.results),
        "completed": [item.completed for item in report.results],
        "stop_reasons": [item.stop_reason for item in report.results],
        "output_count": len(report.outputs),
        "diagnostics": [item.model_dump(mode="json") for item in report.diagnostics],
    }
    ####


def _load_config(path: Path = CONFIG) -> dict[str, Any]:
    """Load and minimally validate the manual execution catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    cases = payload.get("cases", ())
    if not isinstance(cases, list) or len(cases) != 4:
        raise ValueError("Alpha 1 manual execution catalog must contain four cases")
    return payload
    ####


def _fixture_paths(spec: dict[str, Any]) -> tuple[Path, Path, tuple[Path, ...]]:
    """Resolve the canonical synthetic-backed source fixture inputs."""

    fixture_root = FIXTURE_ROOT / str(spec["fixture"]) / "input"
    problem = fixture_root / f"{spec['fixture']}.prb"
    tables = tuple(sorted(fixture_root.glob("*.tbl")))
    if not problem.is_file() or not tables:
        raise FileNotFoundError(f"manual execution fixture is incomplete: {fixture_root}")
    return fixture_root, problem, tables
    ####


def _companion_text(source: str, companion: dict[str, Any]) -> tuple[str, list[dict[str, Any]]]:
    """Apply explicit companion-only source transformations."""

    rendered = source
    transformations: list[dict[str, Any]] = []
    if companion.get("remove_optimization", False):
        start = rendered.find("*optimize")
        end = rendered.find("*egs", start)
        if start < 0 or end < 0:
            raise ValueError("companion requested optimization removal but source block was not found")
        rendered = rendered[:start] + rendered[end:]
        transformations.append({"kind": "remove-block", "block": "optimize"})
    substitutions = companion.get("substitutions", {})
    for placeholder, value in sorted(substitutions.items(), key=lambda item: len(str(item[0])), reverse=True):
        pattern = rf"(?<![A-Za-z0-9_.-]){re.escape(str(placeholder))}(?![A-Za-z0-9_.-])"
        rendered, count = re.subn(pattern, str(value), rendered)
        if count == 0:
            raise ValueError(f"companion substitution did not match {placeholder!r}")
        transformations.append({"kind": "substitution", "placeholder": placeholder, "value": value, "count": count})
    return rendered, transformations
    ####


def _materialize_companion(
    spec: dict[str, Any],
    temporary: Path,
) -> tuple[Path, tuple[Path, ...], list[dict[str, Any]]]:
    """Materialize a declared companion problem and its table overrides."""

    fixture_root, source_problem, source_tables = _fixture_paths(spec)
    companion = dict(spec.get("companion", {}))
    temporary.mkdir(parents=True, exist_ok=True)
    rendered, transformations = _companion_text(source_problem.read_text(encoding="utf-8"), companion)
    problem = temporary / source_problem.name
    problem.write_text(rendered, encoding="utf-8")
    override_by_file = {str(item["file"]): item for item in companion.get("table_overrides", ())}
    tables: list[Path] = []
    for source_table in source_tables:
        text = source_table.read_text(encoding="utf-8")
        override = override_by_file.get(source_table.name)
        if override is not None:
            before = str(override["find"])
            after = str(override["replace"])
            if before not in text:
                raise ValueError(f"companion table override did not match {before!r} in {source_table.name}")
            text = text.replace(before, after, 1)
            transformations.append(
                {
                    "kind": "table-override",
                    "file": source_table.name,
                    "find": before,
                    "replace": after,
                    "reason": override.get("reason"),
                }
            )
        destination = temporary / source_table.name
        destination.write_text(text, encoding="utf-8")
        tables.append(destination)
    return problem, tuple(tables), transformations
    ####


def _run(problem: Path, tables: tuple[Path, ...], output: Path, max_steps: int) -> dict[str, Any]:
    """Run one materialized input through the common TAORYX runner."""

    return _report_summary(run_files(problem, tables, output_dir=output, max_steps=max_steps))
    ####


def execute_case(spec: dict[str, Any], max_steps: int) -> dict[str, Any]:
    """Run one source fixture and its declared companion profile."""

    _, source_problem, source_tables = _fixture_paths(spec)
    companion = dict(spec.get("companion", {}))
    with tempfile.TemporaryDirectory(prefix=f"taoryx-alpha1-{spec['id']}-") as temporary:
        work = Path(temporary)
        source_execution = _run(source_problem, source_tables, work / "source", max_steps)
        companion_problem, companion_tables, transformations = _materialize_companion(spec, work / "companion") if companion else (source_problem, source_tables, [])
        companion_execution = _run(companion_problem, companion_tables, work / "companion-output", max_steps)
    return {
        "id": spec["id"],
        "source_problem": spec["source_problem"],
        "source_problem_sha256": _sha256(ROOT / str(spec["source_problem"])),
        "execution_fixture": f"tests/fixtures/taos_e2e_v23/cases/positive/{spec['fixture']}",
        "execution_problem_sha256": _sha256(source_problem),
        "source_section": spec["source_section"],
        "mode": "3dof",
        "claim": "bounded TAORYX route composition only; not historical TAOS 96.0 output equivalence or intercept performance",
        "source_execution": source_execution,
        "companion_execution": companion_execution,
        "companion_transformations": transformations,
        "optimization_qualification": "pass" if source_execution["status"] == "pass" else "blocked",
    }
    ####


def build_report(max_steps: int, config_path: Path = CONFIG) -> dict[str, Any]:
    """Execute all manual families and return a reproducible summary."""

    catalog = _load_config(config_path)
    cases = [execute_case(spec, max_steps) for spec in catalog["cases"]]
    companion_passing = sum(item["companion_execution"]["status"] == "pass" for item in cases)
    source_passing = sum(item["source_execution"]["status"] == "pass" for item in cases)
    optimization_passing = sum(item["optimization_qualification"] == "pass" for item in cases)
    return {
        "schema_version": 2,
        "release": "taoryx-alpha-1",
        "kind": "manual-example-execution",
        "configuration": "verification/alpha1_manual_execution.yaml",
        "claim_boundary": catalog["claim_boundary"],
        "runner": "taoryx.runtime.runner.run_files",
        "max_steps": max_steps,
        "summary": {
            "families": len(cases),
            "source_passing": source_passing,
            "companion_passing": companion_passing,
            "optimization_passing": optimization_passing,
            "release_status": "pass" if companion_passing == len(cases) else "partial",
            "optimization_status": "pass" if optimization_passing == len(cases) else "partial",
        },
        "cases": cases,
    }
    ####


def main() -> int:
    """Write the four-family manual execution report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-steps", type=int, default=300_000)
    args = parser.parse_args()
    report = build_report(args.max_steps, args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report["summary"], indent=2))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
