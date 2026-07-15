from __future__ import annotations

from pathlib import Path

from taoryx.scenario import ScenarioCompiler
from taoryx.visualization import render_run_artifact_html, render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[2]


def test_run_artifact_html_records_missing_channels_without_source_parsing(tmp_path: Path) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
    )
    artifact = scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0]
    output = render_run_artifact_html(
        artifact,
        tmp_path / "run.html",
        channels=("position.altitude.geodetic", "not.a.real.channel"),
    )

    text = output.read_text(encoding="utf-8")
    assert "position.altitude.geodetic" in text
    assert "channel unavailable" in text
    assert scenario.identity in text
    ####


def test_run_artifact_static_plots_use_normalized_channels(tmp_path: Path) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
    )
    artifact = scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0]
    plots = render_run_artifact_plots(
        artifact,
        tmp_path / "plots",
        vehicle_id="1",
        channels=("position.altitude.geodetic", "not.a.real.channel"),
    )

    assert plots
    assert all(path.suffix == ".png" and path.exists() for path in plots)
    manifest = (tmp_path / "plots" / "plot-manifest.json").read_text(encoding="utf-8")
    assert scenario.identity in manifest
    assert "channel unavailable" in manifest
    ####
