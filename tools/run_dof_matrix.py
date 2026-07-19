"""Execute the reusable 3-DOF/6-DOF matrix and write acceptance evidence."""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Iterable
from pathlib import Path
from typing import Any

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "tests/fixtures/dof_robustness_v1/manifest.json"
AERO = ROOT / "examples/showcases/california_to_hawaii/aero.tbl"
####


def _histories(report: Any) -> list[Any]:
    return [state for result in report.results for history in result.states.values() for state in history]
####


def _numeric_extrema(histories: Iterable[Any], names: tuple[str, ...]) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for name in names:
        values = [float(state.named[name]) for state in histories if name in state.named and math.isfinite(float(state.named[name]))]
        if values:
            result[name] = {"minimum": min(values), "maximum": max(values)}
    return result
####


def _summary(case: dict[str, Any], report: Any, model: str) -> dict[str, Any]:
    histories = _histories(report)
    final = histories[-1] if histories else None
    return {
        "id": case["id"],
        "model": model,
        "claim": case.get("claim"),
        "kind": case.get("kind", "baseline"),
        "reference_3dof": case.get("reference_3dof"),
        "path": case.get("path", case["id"]),
        "exit_code": report.exit_code,
        "completed": bool(report.results) and all(result.completed for result in report.results),
        "diagnostics": [{"code": item.code, "message": item.message} for item in report.diagnostics],
        "vehicle_count": len(report.results[0].states) if report.results else 0,
        "state_count": len(histories),
        "final_time_s": None if final is None else final.time,
        "final": {} if final is None else {name: final.named[name] for name in ("alt", "vel", "mach", "aero_alpha_deg", "aero_sideslip_deg", "range_to_target_m", "pro_nav_los_range_m", "attitude_controller_saturated") if name in final.named},
        "extrema": _numeric_extrema(
            histories,
            ("alt", "vel", "mach", "aero_alpha_deg", "aero_sideslip_deg", "wx", "wy", "wz", "heat_load", "peak_heat_rate", "pro_nav_acceleration_m_s2", "pro_nav_achieved_aero_acceleration_m_s2", "attitude_controller_saturated"),
        ),
    }
####


def _render_plots(report: Any, directory: Path) -> list[str]:
    channels = (
        "position.altitude.geodetic", "kinematics.speed", "aero.airspeed", "aero.dynamic_pressure",
        "aero.force.body.x", "aero.force.body.y", "aero.force.body.z", "aero.moment.body.x",
        "aero.moment.body.y", "aero.moment.body.z", "guidance.pro-nav-acceleration",
        "guidance.pro-nav-achieved-aero-acceleration", "thermal.heat-rate", "attitude.roll",
        "attitude.pitch", "attitude.yaw",
    )
    return [str(path) for artifact in report.artifacts for path in render_run_artifact_plots(artifact, directory, vehicle_id="1", channels=channels)]
####


def _run_three_dof(manifest: dict[str, Any], *, render_plots: bool = False) -> list[dict[str, Any]]:
    sys.path.insert(0, str(ROOT / "tests/e2e"))
    from support.loader import case_directory, load_manifest

    selected = {case["id"]: case for case in manifest["cases"]}
    reports: list[dict[str, Any]] = []
    for case in load_manifest():
        evidence = selected.get(case.id)
        if evidence is None:
            continue
        source = case_directory(case) / "input"
        problem = source / case.problem_file.removeprefix("input/")
        tables = tuple(source / path.removeprefix("input/") for path in case.table_files)
        report = run_files(problem, tables, output_dir=ROOT / "artifacts/dof_robustness_v1" / "3dof" / case.id, max_steps=20_000)
        summary = _summary({**evidence, "path": str(problem.relative_to(ROOT))}, report, "3dof")
        if render_plots:
            summary["plots"] = _render_plots(report, ROOT / "artifacts/dof_robustness_v1" / "plots" / "3dof" / case.id)
        reports.append(summary)
    return reports
####


def _run_six_dof(manifest: dict[str, Any], *, render_plots: bool = False) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    for case in manifest["six_dof_cases"]:
        problem = ROOT / case["path"]
        tables = (ROOT / case["table"],) if case.get("table") else ((AERO,) if case["kind"] == "aero" else ())
        max_steps = 3_000 if case["kind"] in {"route", "wind"} else (2_000 if case["kind"] == "saturation" else (200 if case["kind"] == "ballistic" else 100))
        report = run_files(problem, tables, output_dir=ROOT / "artifacts/dof_robustness_v1" / "6dof" / case["id"], max_steps=max_steps, profile=GrammarProfile.TAORYX)
        summary = _summary(case, report, "6dof")
        if render_plots:
            summary["plots"] = _render_plots(report, ROOT / "artifacts/dof_robustness_v1" / "plots" / "6dof" / case["id"])
        reports.append(summary)
    return reports
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dof", choices=("3", "6", "all"), default="all")
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts/dof_robustness_v1/summary.json")
    parser.add_argument("--plots", action="store_true", help="render Matplotlib PNG artifacts for every matrix case")
    args = parser.parse_args()
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    summaries = []
    if args.dof in {"3", "all"}:
        summaries.extend(_run_three_dof(manifest, render_plots=args.plots))
    if args.dof in {"6", "all"}:
        summaries.extend(_run_six_dof(manifest, render_plots=args.plots))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"manifest": str(MANIFEST_PATH.relative_to(ROOT)), "cases": summaries}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    failed = [case["id"] for case in summaries if case["exit_code"] != 0 or not case["completed"]]
    print(f"Wrote {len(summaries)} matrix summaries to {args.output}")
    if failed:
        print(f"Failed cases: {', '.join(failed)}")
        return 1
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
