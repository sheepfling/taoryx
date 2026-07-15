from __future__ import annotations

from pathlib import Path

from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "mission_families" / "two_stage_ballistic_rocket"


def test_two_stage_ballistic_rocket_runs_from_problem_file(tmp_path: Path) -> None:
    report = run_files(ROOT / "mission.prb", output_dir=tmp_path, max_steps=20_000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    history = result.states["1"]
    assert result.completed
    assert result.stop_reason == "stop_condition"
    assert max(state.named["alt"] for state in history) > 1_000.0
    assert history[-1].named["alt"] < 1.0
    assert max(state.named["mass"] for state in history) > history[-1].named["mass"]
