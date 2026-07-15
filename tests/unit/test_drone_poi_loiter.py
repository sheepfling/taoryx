from __future__ import annotations

from pathlib import Path

from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[1].parent / "examples" / "mission_families" / "drone_poi_loiter"


def test_drone_poi_loiter_mission_runs_from_problem_file(tmp_path: Path) -> None:
    report = run_files(ROOT / "mission.prb", output_dir=tmp_path, max_steps=2_000)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    result = report.results[0]
    assert result.completed
    assert set(result.states) == {"1", "2", "3", "4", "5"}
    drone = result.states["1"]
    segments = {int(state.named["_segment"]) for state in drone}
    assert segments == {1, 2, 3, 4, 5}
    assert drone[-1].time < 80.0
    assert drone[-1].named["relrng[5]"] < 25.0
    for segment, target in ((2, 2), (3, 3), (4, 4)):
        dwell = [
            state
            for state in drone
            if int(state.named["_segment"]) == segment and state.named[f"relrng[{target}]"] < 25.0
        ]
        assert dwell
        assert dwell[-1].time - dwell[0].time >= 8.0
