"""Run a synthetic staged trajectory with a transition and terminal signal."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.outputs import build_run_artifact  # noqa: E402
from taoryx.runtime.common import EventCondition, RuntimeProblem, RuntimeState, RuntimeVehicle  # noqa: E402
from taoryx.runtime.engine import compute_trajectories  # noqa: E402
from taoryx.visualization import render_run_artifact_html  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition/staged"))
    arguments = parser.parse_args()

    def transition(state: RuntimeState) -> RuntimeState:
        vehicle.segment_number = 2
        return RuntimeState(state.time, state.values, state.frame, dict(state.named, stage=2.0), state.value_names, state.segment_endpoints)

    vehicle = RuntimeVehicle(
        "stage-vehicle",
        RuntimeState(0.0, (0.0,), named={"x": 0.0, "stage": 1.0}, value_names=("x",)),
        derivative=lambda state: (1.0,),
        step_size=0.2,
        events=(
            EventCondition("stage-separation", lambda state: state.values[0] - 0.5, "transition", signal="stage-2", source="staged_signals.py:31"),
            EventCondition("terminal", lambda state: state.values[0] - 1.0, "stop", signal="terminal", source="staged_signals.py:32"),
        ),
        event_handlers={"stage-separation": transition},
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=2.0)
    result = compute_trajectories(problem)
    artifact = build_run_artifact("staged-signals", problem, result, events=problem.event_history)
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    artifact.write_json(arguments.output_dir / "staged.json")
    artifact.write_sqlite(arguments.output_dir / "staged.sqlite", run_id="stage-1")
    render_run_artifact_html(artifact, arguments.output_dir / "staged.html", vehicle_id="stage-vehicle")
    print(f"events={len(problem.event_history)}")
    print(f"artifacts={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
