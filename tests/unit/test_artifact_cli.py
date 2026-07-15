from __future__ import annotations

from pathlib import Path

from taoryx.runtime.cli import main
from taoryx.scenario import ScenarioCompiler

ROOT = Path(__file__).resolve().parents[2]


def test_artifact_html_cli_consumes_json_artifact(tmp_path: Path, capsys) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
    )
    artifact_path = tmp_path / "artifact.json"
    scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0].write_json(artifact_path)
    output = tmp_path / "artifact.html"

    assert main(["artifact", "html", str(artifact_path), "--output", str(output)]) == 0
    assert output.exists()
    assert "rendered artifact HTML" in capsys.readouterr().out
    ####


def test_artifact_plot_cli_consumes_json_artifact(tmp_path: Path, capsys) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
    )
    artifact_path = tmp_path / "artifact.json"
    scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0].write_json(artifact_path)
    output = tmp_path / "plots"

    assert main(["artifact", "plot", str(artifact_path), "--output-dir", str(output)]) == 0
    assert list(output.glob("*.png"))
    assert (output / "plot-manifest.json").exists()
    assert "artifact plot" in capsys.readouterr().out
    ####
