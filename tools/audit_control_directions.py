"""Run signed control probes through standard TAORYX problem files."""

from __future__ import annotations

import argparse
import json
import re
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any, cast

import yaml

from taoryx.control_directions import ControlDirectionProbe, Sign, audit_control_directions
from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1]


def _candidate(source: str, control: str, value: float) -> str:
    pattern = rf"(\*runtime control {re.escape(control)}\b[^\n]*?\bdefault=)[0-9.eE+-]+"
    updated, count = re.subn(pattern, rf"\g<1>{value:.16g}", source, count=1)
    if count != 1:
        raise ValueError(f"could not locate runtime control default for {control!r}")
    # Direction probes inspect the initial RHS only; shorten the source
    # scenario without changing its plant, tables, or control declarations.
    updated = re.sub(r"\*when time>[^\n]+ stop", "*when time>0.005 stop", updated, count=1)
    return updated
    ####


def _evaluate(source: str, control: str, value: float, tables: tuple[Path, ...], work: Path) -> dict[str, float]:
    problem = work / f"{control.replace('-', '_')}_{value:+.6g}.prb"
    problem.write_text(_candidate(source, control, value), encoding="utf-8")
    report = run_files(problem, tables, output_dir=work / problem.stem, max_steps=10, integrator="rk4", profile=GrammarProfile.TAORYX)
    if report.exit_code != 0 or not report.results:
        raise RuntimeError([(item.code, item.message) for item in report.diagnostics])
    return dict(report.results[0].states[next(iter(report.results[0].states))][0].named)
    ####


def audit_entry(entry: Mapping[str, Any], work: Path) -> dict[str, object]:
    """Audit one configured source problem and return a JSON report."""

    problem_path = ROOT / str(entry["problem"])
    tables = tuple(ROOT / str(path) for path in entry.get("tables", []))
    source = problem_path.read_text(encoding="utf-8")
    reports: list[dict[str, object]] = []
    for raw_probe in cast(list[Mapping[str, Any]], entry.get("probes", [])):
        expected_sign = raw_probe.get("expected_sign")
        probe = ControlDirectionProbe(
            str(raw_probe["control"]),
            str(raw_probe["response"]),
            float(raw_probe["delta"]),
            cast(Sign, None if expected_sign is None else int(expected_sign)),
            float(raw_probe.get("tolerance", 1e-9)),
            None if raw_probe.get("antisymmetry_tolerance") is None else float(raw_probe["antisymmetry_tolerance"]),
        )
        baseline_value = float(cast(Mapping[str, Any], entry.get("baseline", {})).get(probe.control, 0.0))
        baseline = dict(_evaluate(source, probe.control, baseline_value, tables, work))

        def evaluator(commands: Mapping[str, float]) -> Mapping[str, float]:
            return _evaluate(source, probe.control, commands[probe.control], tables, work)

        result = audit_control_directions(evaluator, {probe.control: baseline_value}, (probe,))[0]
        reports.append({
            "control": probe.control,
            "response": probe.response,
            "baseline_response": baseline.get(probe.response),
            "positive_delta_response": result.positive,
            "negative_delta_response": result.negative,
            "derivative": result.derivative,
            "antisymmetry_error": result.antisymmetry_error,
            "expected_sign": probe.expected_sign,
            "sign_claimed": probe.expected_sign is not None,
            "passed": result.passed,
        })
    return {"vehicle": entry["vehicle"], "problem": str(problem_path.relative_to(ROOT)), "probes": reports}
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=ROOT / "verification/control_direction_contracts.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/control-directions/report.json")
    parser.add_argument("--vehicle", action="append", help="restrict the audit to one or more vehicle IDs")
    args = parser.parse_args()
    catalog = cast(Mapping[str, Any], yaml.safe_load(args.catalog.read_text(encoding="utf-8")))
    with tempfile.TemporaryDirectory(prefix="taoryx-control-directions-") as directory:
        entries = cast(list[Mapping[str, Any]], catalog["vehicles"])
        selected = set(args.vehicle or ())
        if selected:
            entries = [entry for entry in entries if str(entry["vehicle"]) in selected]
        reports = [audit_entry(entry, Path(directory)) for entry in entries]
    probes = [probe for report in reports for probe in cast(list[Mapping[str, Any]], report["probes"])]
    all_pass = all(probe["passed"] for probe in probes)
    all_signs_claimed = all(probe["sign_claimed"] for probe in probes)
    status = "pass" if all_pass and all_signs_claimed else "provisional" if all_pass else "fail"
    payload = {"schema_version": catalog["schema_version"], "vehicles": reports, "status": status}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))
    return 0 if payload["status"] in {"pass", "provisional"} else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
