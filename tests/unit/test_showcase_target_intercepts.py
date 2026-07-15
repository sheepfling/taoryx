from __future__ import annotations

import math
from pathlib import Path

from taoryx.runtime.runner import run_files


def test_standard_3dof_problem_file_hits_stationary_target(tmp_path: Path) -> None:
    root = Path("examples/showcases/3dof_target_intercept")
    report = run_files(root / "mission.prb", output_dir=tmp_path / "3dof", max_steps=3_000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    assert result.completed
    assert result.stop_reason == "stop_condition"
    interceptor = result.states["1"][-1]
    target = result.states["2"][-1]
    miss_m = math.dist(
        (interceptor.named["x"], interceptor.named["y"], interceptor.named["z"]),
        (target.named["x"], target.named["y"], target.named["z"]),
    )
    assert miss_m <= 2.0
    ####
