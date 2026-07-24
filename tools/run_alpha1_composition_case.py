"""Resolve, compile, and execute one metadata-driven Alpha 1 composition case."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

from taoryx.composition import TrajectoryBuilder
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.language.problem_parser import parse_problem_file
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]


def _sha256(path: Path) -> str:
    """Return the SHA-256 digest of one evidence file."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _mapping(payload: object, name: str) -> Mapping[str, Any]:
    """Require a mapping at a metadata boundary."""

    if not isinstance(payload, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return payload
    ####


def _float_mapping(payload: object, name: str) -> dict[str, float]:
    """Normalize a metadata channel mapping to finite numeric values."""

    values = _mapping(payload, name)
    normalized = {str(key): float(value) for key, value in values.items()}
    if any(not value == value or value in {float("inf"), float("-inf")} for value in normalized.values()):
        raise ValueError(f"{name} must contain finite values")
    return normalized
    ####


def _relative(root: Path, path: Path) -> str:
    """Render an evidence path without leaking a machine-local absolute path."""

    return str(path.resolve().relative_to(root.resolve()))
    ####


def _load_case(path: Path) -> Mapping[str, Any]:
    """Load the declarative case manifest."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    values = _mapping(payload, "composition case")
    if values.get("schema_version") != 1:
        raise ValueError("unsupported composition case schema")
    if not isinstance(values.get("segments"), list) or not values["segments"]:
        raise ValueError("composition case requires a non-empty segments list")
    return values
    ####


def _build(case: Mapping[str, Any]) -> TrajectoryBuilder:
    """Build a case through the standard template registry."""

    builder = TrajectoryBuilder(
        str(case["case_id"]),
        vehicle=str(case["vehicle"]),
        family=str(case["family"]),
        source_problem=str(case["source_problem"]),
        output_problem="artifacts/verification/alpha1/composition_case/generated.prb",
        output_manifest="artifacts/verification/alpha1/composition_case/generated-manifest.json",
        output_audit="artifacts/verification/alpha1/composition_case/generated-audit.json",
        runtime_tables=tuple(str(item) for item in case.get("runtime_tables", ())),
        mode=str(case["mode"]),
    )
    for raw_segment in case["segments"]:
        segment = _mapping(raw_segment, "segment")
        optional = {
            key: segment[key]
            for key in ("controller", "actuator_binding", "reference", "dwell_time_s", "completion_condition")
            if key in segment
        }
        builder.use(
            str(segment["template"]),
            str(segment["id"]),
            duration_s=float(segment["duration_s"]),
            target=_float_mapping(segment.get("target", {}), f"{segment['id']}.target"),
            tolerance=_float_mapping(segment.get("tolerance", {}), f"{segment['id']}.tolerance"),
            **optional,
        )
    return builder
    ####


def run_case(root: Path, case_path: Path, report_path: Path) -> dict[str, Any]:
    """Resolve, compile, run, and record one generic composition case."""

    case = _load_case(case_path)
    builder = _build(case)
    scenario = builder.build()
    evaluation = builder.evaluate()
    output_root = root / "artifacts/verification/alpha1/composition_case"
    output_root.mkdir(parents=True, exist_ok=True)
    resolved_path = output_root / "resolved-case.json"
    resolved_path.write_text(json.dumps(scenario.model_dump(mode="json"), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    problem, manifest, audit = builder.compile(root)
    parser_result = parse_problem_file(problem, profile=GrammarProfile.TAORYX)
    table_paths = tuple(root / str(item) for item in case.get("runtime_tables", ()))
    max_steps = int(case.get("max_steps", 100000))
    run_dir = output_root / "run"
    runtime_report = run_files(problem, table_paths, output_dir=run_dir, max_steps=max_steps, profile=GrammarProfile.TAORYX)
    run_artifacts: list[str] = []
    for index, artifact in enumerate(runtime_report.artifacts, start=1):
        artifact_path = run_dir / f"run-artifact-{index}.json"
        artifact.write_json(artifact_path)
        run_artifacts.append(_relative(root, artifact_path))
    parser_diagnostics = [diagnostic.as_dict() for diagnostic in parser_result.diagnostics]
    status = "pass" if evaluation.ok and not parser_diagnostics and runtime_report.exit_code == 0 and run_artifacts else "blocked"
    report = {
        "schema_version": 1,
        "status": status,
        "claim_boundary": "Metadata-driven composition and common runtime evidence; not vehicle certification or historical TAOS compatibility.",
        "case": {
            "case_path": _relative(root, case_path),
            "case_sha256": _sha256(case_path),
            "case_id": str(case["case_id"]),
            "vehicle": str(case["vehicle"]),
            "family": str(case["family"]),
            "mode": str(case["mode"]),
        },
        "composition": {
            "api": "taoryx.composition.TrajectoryBuilder",
            "template_registry": "SegmentCompositionRegistry.standard",
            "bespoke_runner_logic": False,
            "duration_s": builder.duration_s,
            "segment_ids": [segment.id for segment in scenario.segments],
            "evaluation": {"status": evaluation.status, "ok": evaluation.ok, "messages": list(evaluation.messages)},
        },
        "generated": {
            "problem": _relative(root, problem),
            "manifest": _relative(root, manifest),
            "audit": _relative(root, audit),
            "resolved_case": _relative(root, resolved_path),
            "hashes": {name: _sha256(path) for name, path in (("problem", problem), ("manifest", manifest), ("audit", audit), ("resolved_case", resolved_path))},
        },
        "parser": {"profile": GrammarProfile.TAORYX.value, "diagnostics": parser_diagnostics},
        "runtime": {"exit_code": runtime_report.exit_code, "artifacts": run_artifacts, "summary": runtime_report.as_dict()},
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report
    ####


def main() -> int:
    """Run the metadata-driven Alpha 1 composition proof."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", type=Path, default=ROOT / "verification/alpha1_composition_case.yaml")
    parser.add_argument("--report", type=Path, default=ROOT / "artifacts/verification/alpha1/composition_case/report.json")
    args = parser.parse_args()
    report = run_case(ROOT, args.case, args.report)
    print(json.dumps({"status": report["status"], "report": _relative(ROOT, args.report)}, sort_keys=True))
    return 0 if report["status"] == "pass" else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
    ####
