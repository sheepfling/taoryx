"""Apply a frame-aware initial kick and demonstrate the attitude guard."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.scenario import ScenarioCompiler, VelocityImpulse  # noqa: E402
from taoryx.visualization import render_run_artifact_html  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition/initial-kick"))
    arguments = parser.parse_args()
    problem = ROOT / "examples/chapter04/ballistic-reentry.prb"
    table = ROOT / "examples/chapter04/ballistic-reentry.tbl"
    scenario = ScenarioCompiler().compile(
        problem,
        table_paths=(table,),
        patches=(VelocityImpulse(vehicle="1", frame="geodetic", delta_velocity=(0.0, 0.0, 12.0)),),
    )
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    scenario.write_json(arguments.output_dir / "scenario.json")
    artifact = scenario.run(output_dir=arguments.output_dir / "run", max_steps=20_000)[0]
    artifact.write_json(arguments.output_dir / "run.json")
    render_run_artifact_html(artifact, arguments.output_dir / "run.html", vehicle_id="1")

    rejected = False
    try:
        ScenarioCompiler().compile(
            problem,
            table_paths=(table,),
            patches=(VelocityImpulse(vehicle="1", frame="body", delta_velocity=(1.0, 0.0, 0.0)),),
        ).lower()
    except ValueError as error:
        rejected = "attitude provider" in str(error)
    assert rejected, "body-frame kick without attitude provider must be rejected"
    print(f"scenario={scenario.identity}")
    print(f"artifacts={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
