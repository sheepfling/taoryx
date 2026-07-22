"""Run bound native maneuver cases and write classified evidence artifacts."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "verification/vehicle_maneuver_matrix.yaml"
BINDINGS = ROOT / "verification/maneuver_evidence.yaml"
CHANNELS = (
    "taos.altitude_m", "taos.speed_m_s", "taos.range_to_target_m",
    "taos.aero_alpha_deg", "taos.aero_sideslip_deg", "taos.local_pitch_deg",
    "taos.local_roll_deg", "taos.local_heading_deg", "taos.aero_dynamic_pressure_pa",
    "taos.aero_force_body_z_n", "taos.aero_moment_body_y_nm",
)
FINAL_CHANNELS = (
    "altitude_m", "alt", "speed_m_s", "vel", "range_to_target_m",
    "aero_alpha_deg", "aero_sideslip_deg", "mach",
)


def _load(path: Path) -> dict[str, Any]:
    return yaml.safe_load(path.read_text(encoding="utf-8"))
    ####


def _matrix_cases() -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for vehicle in _load(MATRIX)["vehicles"]:
        for case in vehicle["cases"]:
            result[f"{vehicle['id']}/{case['id']}"] = {"vehicle": vehicle["id"], **case}
    return result
    ####


def _binding_fingerprint(binding: dict[str, Any]) -> str:
    """Fingerprint the binding and every source file used by the run."""

    problem = ROOT / str(binding["problem"])
    source = {
        "binding": binding,
        "problem": hashlib.sha256(problem.read_bytes()).hexdigest(),
        "tables": {
            str(path): hashlib.sha256((ROOT / str(path)).read_bytes()).hexdigest()
            for path in binding.get("tables", [])
        },
    }
    return hashlib.sha256(json.dumps(source, sort_keys=True).encode("utf-8")).hexdigest()
    ####


def _run(binding: dict[str, Any], output: Path, plots: bool) -> dict[str, Any]:
    problem = ROOT / str(binding["problem"])
    tables = tuple(ROOT / str(path) for path in binding.get("tables", []))
    report = run_files(problem, tables, output_dir=output / "run", max_steps=int(binding["max_steps"]), integrator="rk4", profile=GrammarProfile.TAORYX)
    histories = [state for result in report.results for history in result.states.values() for state in history]
    final = histories[-1] if histories else None
    completed = bool(report.results) and all(item.completed for item in report.results)
    violations: list[dict[str, float | str]] = []
    limits = dict(binding.get("limits", {}))
    is_point_mass = str(binding["dimension"]).casefold() == "3dof"
    if histories:
        checks = (
            ("max_abs_aero_alpha_deg", "aero_alpha_deg", "abs_max"),
            ("max_abs_aero_sideslip_deg", "aero_sideslip_deg", "abs_max"),
            ("max_speed_m_s", "speed_m_s", "max"),
            ("max_speed_m_s", "vel", "max"),
            ("min_speed_m_s", "speed_m_s", "min"),
            ("min_speed_m_s", "vel", "min"),
            ("min_altitude_m", "altitude_m", "min"),
            ("min_altitude_m", "alt", "min"),
            ("max_altitude_m", "altitude_m", "max"),
            ("max_altitude_m", "alt", "max"),
        )
        for limit_name, channel, operation in checks:
            if limit_name not in limits:
                continue
            values = [float(state.named[channel]) for state in histories if channel in state.named and float(state.named[channel]) == float(state.named[channel])]
            if is_point_mass and channel in {"vel", "alt"}:
                # The manual-compatible point-mass output uses ft/s and ft;
                # evidence limits are expressed in the canonical SI contract.
                values = [value * 0.3048 for value in values]
            if not values:
                continue
            observed = max(abs(value) for value in values) if operation == "abs_max" else (min(values) if operation == "min" else max(values))
            limit = float(limits[limit_name])
            violated = observed > limit if operation in {"abs_max", "max"} else observed < limit
            if violated:
                violations.append({"limit": limit_name, "channel": channel, "observed": observed, "allowed": limit})
    termination_events = [
        event
        for artifact in report.artifacts
        for event in artifact.events
        if str(event.get("signal", "")).casefold() in {"earth-intersection", "runtime-safety"}
    ]
    safety_events = [
        event
        for event in termination_events
        if not (binding.get("allow_ground_termination", False) and str(event.get("signal", "")).casefold() == "earth-intersection")
    ]
    for event in safety_events:
        violations.append({"limit": "runtime-safety", "channel": str(event.get("signal", "runtime-safety")), "observed": float(event.get("time", 0.0)), "allowed": "no safety termination"})
    if completed and not violations:
        status = "completed_ground_terminated" if termination_events and binding.get("allow_ground_termination", False) else "completed"
    else:
        status = "completed_unsafe" if completed else ("failed" if report.exit_code != 0 and not report.results else "incomplete")
    final_values = {} if final is None else {name: final.named[name] for name in FINAL_CHANNELS if name in final.named}
    if is_point_mass and final is not None:
        if "alt" in final.named:
            final_values["altitude_m"] = float(final.named["alt"]) * 0.3048
            final_values["alt_ft"] = float(final.named["alt"])
        if "vel" in final.named:
            final_values["speed_m_s"] = float(final.named["vel"]) * 0.3048
            final_values["vel_ft_s"] = float(final.named["vel"])
    result: dict[str, Any] = {
        "id": binding["id"],
        "case_id": binding.get("case_id", binding["id"]),
        "dimension": binding["dimension"],
        "problem": str(problem.relative_to(ROOT)),
        "tables": [str(path.relative_to(ROOT)) for path in tables],
        "exit_code": report.exit_code,
        "stop_reason": [item.stop_reason for item in report.results],
        "termination_events": termination_events,
        "completed": completed,
        "status": status,
        "limits": limits,
        "violations": violations,
        "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
        "state_count": len(histories),
        "final_time_s": None if final is None else final.time,
        "final": final_values,
        "fingerprint": _binding_fingerprint(binding),
    }
    if plots:
        paths = [str(path) for artifact in report.artifacts for path in render_run_artifact_plots(artifact, output / "plots", vehicle_id="1", channels=CHANNELS)]
        result["plots"] = paths
    (output / "summary.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "run-report.json").write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/vehicle-maneuver-matrix")
    parser.add_argument("--plots", action="store_true")
    parser.add_argument("--status", default="available,in_progress", help="matrix statuses to execute")
    parser.add_argument("--only", default="", help="optional comma-separated case ids to execute")
    parser.add_argument("--aggregate-only", action="store_true", help="rebuild the consolidated report from valid existing summaries")
    args = parser.parse_args()
    if args.plots:
        mpl_config = ROOT / "artifacts" / ".mplconfig"
        mpl_config.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(mpl_config))
    selected = {item.strip() for item in args.status.split(",") if item.strip()}
    only = {item.strip() for item in args.only.split(",") if item.strip()}
    matrix = _matrix_cases()
    bindings = _load(BINDINGS)["cases"]
    reports: list[dict[str, Any]] = []
    if not args.aggregate_only:
        for binding in bindings:
            case = matrix.get(str(binding.get("case_id", binding["id"])))
            if case is None:
                reports.append({"id": binding["id"], "status": "unmapped"})
                continue
            if case["status"] not in selected:
                continue
            if only and str(binding["id"]) not in only:
                continue
            reports.append(_run(binding, args.output / str(binding["id"]).replace("/", "-"), args.plots))
    # Consolidate valid summaries from prior invocations. This makes a partial
    # rerun additive instead of replacing the evidence view with only the last
    # selected bindings.
    consolidated: dict[str, dict[str, Any]] = {}
    for binding in bindings:
        binding_id = str(binding["id"])
        summary_path = args.output / binding_id.replace("/", "-") / "summary.json"
        if not summary_path.is_file():
            continue
        try:
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if summary.get("fingerprint") == _binding_fingerprint(binding):
            consolidated[binding_id] = summary
    reports = list(consolidated.values())
    bound = {str(binding.get("case_id", binding["id"])) for binding in bindings}
    reports.extend({"id": identifier, "status": case["status"], "objective": case["objective"]} for identifier, case in matrix.items() if identifier not in bound)
    coverage = []
    for identifier, case in matrix.items():
        required = [str(case["dimension"])] if isinstance(case["dimension"], str) else [str(item) for item in case["dimension"]]
        matching = [binding for binding in bindings if str(binding.get("case_id", binding["id"])) == identifier]
        bound_dimensions = sorted({str(binding["dimension"]) for binding in matching})
        observed = [summary for summary in reports if str(summary.get("case_id", summary.get("id", ""))) == identifier]
        passing = [summary for summary in observed if summary.get("status") in {"completed", "completed_ground_terminated"}]
        passing_dimensions = sorted({str(summary["dimension"]) for summary in passing})
        row_status = "pass" if passing else ("unsafe" if any(summary.get("status") == "completed_unsafe" for summary in observed) else "incomplete" if observed else "unrun")
        coverage.append({
            "id": identifier,
            "matrix_status": case["status"],
            "required_dimensions": required,
            "bound_dimensions": bound_dimensions,
            "missing_dimensions": sorted(set(required) - set(bound_dimensions)),
            "binding_ids": [str(binding["id"]) for binding in matching],
            "passing_dimensions": passing_dimensions,
            "passing_binding_ids": [str(summary["id"]) for summary in passing],
            "row_status": row_status,
        })
    payload = {
        "matrix": _load(MATRIX)["id"],
        "bindings": _load(BINDINGS)["id"],
        "cases": reports,
        "coverage": coverage,
        "coverage_summary": {
            "matrix_cases": len(coverage),
            "fully_bound_cases": sum(not item["missing_dimensions"] for item in coverage),
            "dimension_gaps": sum(bool(item["missing_dimensions"]) for item in coverage),
            "actionable_dimension_gaps": sum(bool(item["missing_dimensions"]) and item["matrix_status"] != "blocked" for item in coverage),
            "blocked_cases_with_missing_dimensions": sum(bool(item["missing_dimensions"]) and item["matrix_status"] == "blocked" for item in coverage),
            "passing_matrix_rows": sum(item["row_status"] == "pass" for item in coverage),
            "matrix_rows": len(coverage),
        },
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "matrix-report.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "cases": len(reports)}, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
