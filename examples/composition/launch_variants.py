"""Build one reproducible, identity-linked TAORYX composition variant."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.scenario import InitialFrameState, ScenarioCompiler, VelocityImpulse
from taoryx.visualization import render_run_artifact_html


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition"))
    arguments = parser.parse_args()
    root = ROOT
    output = arguments.output_dir
    scenario = ScenarioCompiler().compile(
        root / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(root / "examples/chapter04/ballistic-reentry.tbl",),
        seed=1729,
        patches=(
            InitialFrameState(
                vehicle="1",
                frame="geodetic",
                position=(-80.6, 28.5, 30_000.0),
                velocity=(100.0, 5.0, 90.0),
            ),
            VelocityImpulse(vehicle="1", frame="geodetic", delta_velocity=(0.0, 0.0, 12.0)),
        ),
    )
    scenario.write_json(output / "scenario.json")
    artifact = scenario.run(output_dir=output / "run", max_steps=20_000)[0]
    artifact.write_json(output / "run.json")
    artifact.write_csv(output / "run.csv", vehicle_id="1")
    artifact.write_sqlite(output / "run.sqlite", run_id="case-1")
    render_run_artifact_html(artifact, output / "run.html", vehicle_id="1")
    print(f"scenario={scenario.identity}")
    print(f"artifacts={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
