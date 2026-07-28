"""Replay a timestamped SWIL/HWIL rotational truth stream through hybrid IMU."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.runtime import (  # noqa: E402
    RotationalTruthRecorder,
    RotationalTruthReplay,
    RuntimeProblem,
    RuntimeState,
    RuntimeVehicle,
    SensorScenarioSpec,
    TranslationTruthProvider,
    attach_sensor_scenario,
)
from taoryx.runtime.engine import compute_trajectories  # noqa: E402

DEFAULT_REPLAY = ROOT / "examples/sensors/rotational_truth_replay_v1.jsonl"


def run(replay_path: Path, output_dir: Path) -> dict[str, object]:
    replay = RotationalTruthReplay.from_jsonl(replay_path)
    recorder = RotationalTruthRecorder(replay, source="jsonl-swil-hwil-replay")
    translation = TranslationTruthProvider(3.986004418e14, 0.0)
    vehicle = RuntimeVehicle(
        "vehicle",
        RuntimeState(
            0.0,
            (6_378_137.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            frame="ecic",
            value_names=("x", "y", "z", "xdt", "ydt", "zdt"),
        ),
        derivative=lambda state: (0.0,) * len(state.values),
        step_size=0.1,
        truth_provider=translation,
    )
    problem = RuntimeProblem({vehicle.name: vehicle}, final_time=0.3)
    spec = SensorScenarioSpec(
        sensor_name="imu",
        vehicle_name=vehicle.name,
        scenario_id="rotational_truth_replay_v1",
        provider="ideal",
        truth_mode="hybrid-6dof",
        cadence_s=0.1,
        estimator_modes=("dead-reckoning",),
    )
    runtime = attach_sensor_scenario(problem, spec, rotational_truth_provider=recorder)
    result = compute_trajectories(problem, max_steps=10)
    runtime.finalize(completed=result.completed, stop_reason=result.stop_reason, max_steps=10)
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime.write_artifacts(output_dir, 1)
    recorder.write_request_log(output_dir / "rotational-request-log.jsonl")
    artifact = runtime.artifact()
    report = {
        "artifact_id": "taoryx.rotational-truth-replay-demo.v1",
        "completed": result.completed,
        "stop_reason": result.stop_reason,
        "replay_contract": replay.contract,
        "recorder_contract": recorder.contract,
        "request_count": len(recorder.records),
        "accepted_request_count": sum(record["status"] == "accepted" for record in recorder.records),
        "sensor_execution": artifact,
    }
    (output_dir / "replay-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--replay", type=Path, default=DEFAULT_REPLAY)
    parser.add_argument("--output-dir", type=Path, required=True)
    arguments = parser.parse_args()
    report = run(arguments.replay.resolve(), arguments.output_dir.resolve())
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["completed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
