from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.runtime.runner import run_files

from .support.loader import case_directory, load_manifest

ROOT = Path(__file__).resolve().parents[2]
MATRIX = json.loads((ROOT / "tests/fixtures/dof_robustness_v1/manifest.json").read_text(encoding="utf-8"))
MATRIX_BY_ID = {item["id"]: item for item in MATRIX["cases"]}
BASELINE_CASES = tuple(case for case in load_manifest() if case.id in MATRIX_BY_ID)


@pytest.mark.dof3
@pytest.mark.parametrize("case", BASELINE_CASES, ids=lambda item: item.id)
def test_three_dof_robustness_baseline(case, tmp_path) -> None:
    """Run the first 3-DOF matrix before introducing rigid-body coupling."""

    source = case_directory(case) / "input"
    problem = source / case.problem_file.removeprefix("input/")
    tables = tuple(source / path.removeprefix("input/") for path in case.table_files)
    report = run_files(problem, tables, output_dir=tmp_path / case.id, max_steps=20000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and all(result.completed for result in report.results)
    evidence = MATRIX_BY_ID[case.id]
    assert len(report.results[0].states) >= evidence.get("min_vehicles", 1)
    for history in report.results[0].states.values():
        assert len(history) >= 2
        assert history[-1].time >= evidence.get("min_duration_s", 0.0)
        assert all(value == value and abs(value) != float("inf") for state in history for value in state.values)
    invariant = evidence.get("invariant")
    first_history = next(iter(report.results[0].states.values()))
    if invariant == "altitude_increases":
        assert first_history[-1].named["alt"] > first_history[0].named["alt"]
    elif invariant == "altitude_decreases":
        assert first_history[-1].named["alt"] < first_history[0].named["alt"]
    elif invariant == "guidance_solved":
        assert any(state.named.get("_guidance_solved", 0.0) for state in first_history)
    elif invariant == "guidance_demand":
        assert any(abs(state.named.get("_guidance_ax", 0.0)) + abs(state.named.get("_guidance_ay", 0.0)) + abs(state.named.get("_guidance_az", 0.0)) > 0.0 for state in first_history)
    ####
