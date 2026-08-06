"""Run the canonical two-step California–Hawaii interactive witness."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.runtime.interactive import ControlSpec, InteractiveSession
from taoryx.runtime.program import LoadedProgram

ROOT = Path(__file__).resolve().parents[3]


def run(output_dir: Path, *, duration_s: float = 10.0) -> dict[str, object]:
    """Execute two bounded external steps and persist both artifact views."""

    output_dir.mkdir(parents=True, exist_ok=True)
    program = LoadedProgram.load(
        ROOT / "examples/showcases/california_to_hawaii/mission.prb",
        (ROOT / "examples/showcases/california_to_hawaii/aero.tbl",),
        profile="taoryx",
    )
    session = InteractiveSession(
        program.case(),
        controls=(
            ControlSpec("throttle", unit="fraction", lower=0.0, upper=1.0),
            ControlSpec("alpha-deg", unit="deg", lower=-20.0, upper=20.0),
            ControlSpec("bank-deg", unit="deg", lower=-180.0, upper=180.0),
        ),
    )
    snapshots = (
        session.step(duration_s, {"throttle": 1.0, "alpha-deg": 0.0, "bank-deg": 0.0}),
        session.step(duration_s, {"throttle": 0.8, "alpha-deg": 2.0, "bank-deg": 5.0}),
    )
    interactive_path = session.artifact().write_json(output_dir / "interactive-artifact.json")
    run_artifact = session.to_run_artifact()
    run_artifact_path = run_artifact.write_json(output_dir / "run-artifact.json")
    payload = {
        "schema": "taoryx.product-two-interactive-witness/v1alpha1",
        "status": session.status.value,
        "snapshot_count": len(snapshots),
        "accepted_intervals": [
            {"start_s": snapshot.time_start, "end_s": snapshot.time_end, "status": snapshot.status.value}
            for snapshot in snapshots
        ],
        "interactive_artifact": str(interactive_path),
        "run_artifact": str(run_artifact_path),
        "claim_boundary": (
            "Two bounded external steps and their normalized telemetry over the synthetic route; no target-hit, "
            "guidance, physical-effector, or validated flight-design claim."
        ),
    }
    (output_dir / "interactive-summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
    ####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=10.0)
    arguments = parser.parse_args()
    print(json.dumps(run(arguments.output_dir, duration_s=arguments.duration_s), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
