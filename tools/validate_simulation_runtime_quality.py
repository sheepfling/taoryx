"""Build the Simulation Runtime M4 numerical-quality report."""

from __future__ import annotations

import argparse
import json
import re
import tempfile
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from taoryx.runtime.runner import run_files
from taoryx.simulation_runtime_bundle import build_simulation_runtime_bundle
from taoryx.simulation_runtime_catalog import ROOT, SimulationRuntimeScenario, load_simulation_runtime_catalog
from taoryx.simulation_runtime_quality import (
    SimulationRuntimeNumericalQualityReport,
    SimulationRuntimeQualityCheck,
    evaluate_simulation_runtime_quality,
    validate_simulation_runtime_quality_catalog,
)

_DT_ASSIGNMENT = re.compile(r"(?<![A-Za-z0-9_])dt(?P<separator>\s*=\s*)(?P<value>[0-9]+(?:\.[0-9]*)?(?:[eE][+-]?[0-9]+)?)")
DEFAULT_OUTPUT = ROOT / "artifacts/verification/simulation_runtime_quality/report.json"


def build_simulation_runtime_quality_suite(
    *,
    catalog_path: str | Path | None = None,
    execute: bool = False,
    selected_scenarios: tuple[str, ...] = (),
) -> dict[str, object]:
    """Build one report containing every canonical scenario disposition."""

    catalog = load_simulation_runtime_catalog(catalog_path or ROOT / "verification/simulation_runtime_scenario_catalog.yaml")
    catalog_errors = validate_simulation_runtime_quality_catalog(catalog.scenarios)
    if catalog_errors:
        raise ValueError("invalid Simulation Runtime M4 catalog: " + "; ".join(catalog_errors))
    selected = set(selected_scenarios)
    executed: list[str] = []
    reports: list[SimulationRuntimeNumericalQualityReport] = []
    with tempfile.TemporaryDirectory(prefix="taoryx-simulation-runtime-quality-") as temporary:
        temporary_root = Path(temporary)
        for scenario in catalog.scenarios:
            if not execute or (selected and scenario.id not in selected):
                reports.append(evaluate_simulation_runtime_quality(scenario))
                continue
            executed.append(scenario.id)
            try:
                primary, repeat, coarse, fine = _execute_scenario(scenario, temporary_root / scenario.id)
            except (OSError, RuntimeError, TypeError, ValueError) as error:
                report = evaluate_simulation_runtime_quality(scenario)
                reports.append(_append_execution_failure(report, str(error)))
            else:
                reports.append(
                    evaluate_simulation_runtime_quality(
                        scenario,
                        primary=primary,
                        repeat=repeat,
                        coarse=coarse,
                        fine=fine,
                    )
                )
    scenario_reports = [report.as_dict() for report in reports]
    return {
        "schema": "taoryx.simulation-runtime-quality-suite/v1alpha1",
        "schema_version": 1,
        "status": _suite_disposition(reports),
        "executed_scenarios": executed,
        "scenarios": scenario_reports,
        "claim_boundary": (
            "M4 reports finite telemetry, event ordering, declared continuity and invariants, fixed-input repeatability, "
            "and selected step/integrator refinement. A pass is numerical evidence for the declared lane only; it does "
            "not promote a visual, nominal, pseudo-6DOF, or source-history result into a higher-fidelity claim."
        ),
    }
    ####


def _execute_scenario(
    scenario: SimulationRuntimeScenario,
    root: Path,
) -> tuple[Any, Any, Any, Any]:
    """Execute one selected scenario and return primary/repeat/refinement artifacts."""

    root.mkdir(parents=True, exist_ok=True)
    if scenario.entrypoint == "source":
        primary = _run_source(scenario, root / "primary", multiplier=1.0, integrator=scenario.integrator or "rk4")
        repeat = _run_source(scenario, root / "repeat", multiplier=1.0, integrator=scenario.integrator or "rk4")
        if not scenario.quality.refinement.enabled:
            return primary, repeat, None, None
        refinement = scenario.quality.refinement
        coarse = _run_source(scenario, root / "coarse", multiplier=refinement.coarse_step_multiplier, integrator=refinement.coarse_integrator)
        fine = _run_source(scenario, root / "fine", multiplier=refinement.fine_step_multiplier, integrator=refinement.fine_integrator)
        return primary, repeat, coarse, fine
    if scenario.quality.artifact_required or scenario.entrypoint in {"interactive", "composition"}:
        build_simulation_runtime_bundle(scenario, root / "primary", root=ROOT, run=True, plots=False)
        build_simulation_runtime_bundle(scenario, root / "repeat", root=ROOT, run=True, plots=False)
        primary = _read_bundle_artifact(root / "primary")
        repeat = _read_bundle_artifact(root / "repeat")
        if primary is None and repeat is None:
            return None, None, None, None
        return primary, repeat, None, None
    build_simulation_runtime_bundle(scenario, root / "primary", root=ROOT, run=True, plots=False)
    build_simulation_runtime_bundle(scenario, root / "repeat", root=ROOT, run=True, plots=False)
    return None, None, None, None
    ####


def _run_source(scenario: SimulationRuntimeScenario, output: Path, *, multiplier: float, integrator: str) -> Any:
    problem = scenario.input_path(next(item for item in scenario.inputs if item.kind == "problem"), root=ROOT)
    tables = tuple(scenario.input_path(item, root=ROOT) for item in scenario.inputs if item.kind == "table")
    if multiplier == 1.0:
        problem_path = problem
    else:
        problem_path = output / problem.name
        problem_path.parent.mkdir(parents=True, exist_ok=True)
        problem_path.write_text(_scale_step_sizes(problem.read_text(encoding="utf-8"), multiplier), encoding="utf-8")
    report = run_files(
        problem_path,
        tables,
        output_dir=output,
        max_steps=20000,
        integrator=integrator,
        seed=scenario.seed,
        profile=scenario.profile,
    )
    if report.exit_code != 0 or not report.artifacts:
        detail = "; ".join(diagnostic.message for diagnostic in report.diagnostics) or f"source run returned {report.exit_code}"
        raise RuntimeError(f"{scenario.id}: {detail}")
    return report.artifacts[0]
    ####


def _read_bundle_artifact(root: Path) -> Any:
    candidates = (root / "run" / "run-artifact.json", root / "run" / "interactive-artifact.json")
    for candidate in candidates:
        if candidate.is_file():
            from taoryx.outputs import RunArtifact

            return RunArtifact.model_validate_json(candidate.read_text(encoding="utf-8"))
    return None
    ####


def _scale_step_sizes(source: str, multiplier: float) -> str:
    if multiplier <= 0.0:
        raise ValueError("step-size multiplier must be positive")
    replacements = 0

    def replace(match: re.Match[str]) -> str:
        nonlocal replacements
        replacements += 1
        value = float(match.group("value")) * multiplier
        return f"dt{match.group('separator')}{value:.12g}"

    updated = _DT_ASSIGNMENT.sub(replace, source)
    if replacements == 0:
        raise ValueError("selected refinement source contains no dt assignments")
    return updated
    ####


def _append_execution_failure(report: SimulationRuntimeNumericalQualityReport, message: str) -> SimulationRuntimeNumericalQualityReport:
    check = SimulationRuntimeQualityCheck(
        id="execution",
        disposition="bounded-failure",
        required=True,
        message="selected M4 execution did not produce a usable artifact",
        observed={"error": message},
    )
    return report.model_copy(update={"disposition": "bounded-failure", "checks": (*report.checks, check)})
    ####


def _suite_disposition(reports: Sequence[SimulationRuntimeNumericalQualityReport]) -> str:
    dispositions = {report.disposition for report in reports}
    if "bounded-failure" in dispositions:
        return "bounded-failure"
    if "blocked" in dispositions:
        return "blocked"
    if "development" in dispositions:
        return "development"
    return "pass"
    ####


def main(argv: list[str] | None = None) -> int:
    """Validate the M4 catalog and optionally execute selected scenarios."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, default=ROOT / "verification/simulation_runtime_scenario_catalog.yaml")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--execute", action="store_true", help="execute selected canonical scenarios")
    parser.add_argument("--scenario", action="append", default=[], help="scenario ID to execute; repeat for a selected set")
    args = parser.parse_args(argv)
    report = build_simulation_runtime_quality_suite(
        catalog_path=args.catalog,
        execute=args.execute,
        selected_scenarios=tuple(args.scenario),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 1 if report["status"] == "bounded-failure" else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
