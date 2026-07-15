"""Reconcile one run through JSON, SQLite, CSV, text, and HTML sinks."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.outputs import RunArtifact  # noqa: E402
from taoryx.scenario import ScenarioCompiler  # noqa: E402
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition/round-trip"))
    arguments = parser.parse_args()
    scenario = ScenarioCompiler().compile(ROOT / "examples/chapter04/ballistic-reentry.prb", table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",))
    artifact = scenario.run(output_dir=arguments.output_dir / "run", max_steps=20_000)[0]
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = artifact.write_json(arguments.output_dir / "run.json")
    restored = RunArtifact.model_validate_json(json_path.read_text(encoding="utf-8"))
    assert restored.scenario_identity == scenario.identity
    artifact.write_sqlite(arguments.output_dir / "run.sqlite", run_id="case-1")
    artifact.write_csv(arguments.output_dir / "run.csv", vehicle_id="1")
    artifact.format_text(max_rows=2)
    render_run_artifact_html(restored, arguments.output_dir / "run.html", vehicle_id="1")
    render_run_artifact_plots(restored, arguments.output_dir / "plots", vehicle_id="1")
    print(f"identity={scenario.identity}")
    print(f"artifacts={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
