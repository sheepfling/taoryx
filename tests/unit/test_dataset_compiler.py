from __future__ import annotations

import json
from itertools import product
from pathlib import Path

import pytest

from taoryx.datasets import compile_dataset, flatten_rectangular_grid
from taoryx.fixtures.table_example_artifacts import build_run_plot_series
from taoryx.language.ingest import ingest_file
from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables
from taoryx.runtime.runner import run_files
from taoryx.tables import interpolate_nd, prepare_table
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "datasets" / "generic_cruise_demo"

pytestmark = pytest.mark.table


def test_flatten_rectangular_grid_uses_last_axis_fastest() -> None:
    axes = {"alt": (0.0, 10.0), "mach": (0.3, 0.8), "power": (0.0, 1.0)}
    samples = {
        coordinate: float(index)
        for index, coordinate in enumerate(product(axes["alt"], axes["mach"], axes["power"]))
    }

    assert flatten_rectangular_grid(("alt", "mach", "power"), axes, samples) == tuple(float(index) for index in range(8))
    ####


def test_generic_cruise_compiles_and_round_trips_source_nodes(tmp_path: Path) -> None:
    generated = compile_dataset(ROOT / "dataset.yaml", tmp_path)
    assert sorted(path.name for path in generated.values()) == sorted(
        ["demo-cg.tbl", "demo-clean-cd.tbl", "demo-clean-cl.tbl", "demo-flap-dcd.tbl", "demo-flap-dcl.tbl", "demo-jet-mdot.tbl", "demo-jet-thrust.tbl", "demo-speedbrake-dcd.tbl", "demo-wind-down.tbl", "demo-wind-heading.tbl", "demo-wind-speed.tbl", "provenance.json"]
    )

    provenance = json.loads(generated["provenance"].read_text(encoding="utf-8"))
    assert provenance["dataset_id"] == "generic_cruise_demo"
    assert provenance["generator"]["flight_qualified"] is False
    assert len(provenance["source_hashes"]) == 6
    assert provenance["conventions"]["wind_heading"] == "to"

    for path in generated.values():
        if path.suffix != ".tbl":
            continue
        ingested = ingest_file(path)
        assert ingested.valid, path
        runtime_table = next(iter(lower_tables(parse_table_file(path)).values()))
        assert runtime_table.prepared is not None, path
        for coordinate in product(*runtime_table.prepared.axes):
            assert interpolate_nd(runtime_table.prepared, coordinate) in runtime_table.prepared.values
    ####


def test_generic_cruise_rejects_incomplete_source_grid(tmp_path: Path) -> None:
    source = ROOT / "source" / "clean_aero.csv"
    broken = tmp_path / "broken.csv"
    broken.write_text("\n".join(source.read_text(encoding="utf-8").splitlines()[:-1]) + "\n", encoding="utf-8")
    manifest = (ROOT / "dataset.yaml").read_text(encoding="utf-8").replace("source/clean_aero.csv", "broken.csv")
    manifest_path = tmp_path / "dataset.yaml"
    manifest_path.write_text(manifest, encoding="utf-8")

    with pytest.raises(ValueError, match="incomplete"):
        compile_dataset(manifest_path, tmp_path / "build")
    ####


def test_generic_cruise_problem_runs_against_compiled_tables(tmp_path: Path) -> None:
    generated = compile_dataset(ROOT / "dataset.yaml", tmp_path / "tables")
    table_files = tuple(path for name, path in generated.items() if name != "provenance")
    report = run_files(
        Path(__file__).resolve().parents[2] / "examples" / "data_driven" / "generic_cruise.prb",
        table_files,
        output_dir=tmp_path / "run",
        max_steps=100,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and all(result.completed for result in report.results)
    assert report.artifacts
    ####


@pytest.mark.artifact
def test_generic_cruise_artifact_uses_standard_png_visualization(artifact_dir: Path) -> None:
    generated = compile_dataset(ROOT / "dataset.yaml", artifact_dir / "compiled-tables")
    table_files = tuple(path for name, path in generated.items() if name != "provenance")
    report = run_files(
        Path(__file__).resolve().parents[2] / "examples" / "data_driven" / "generic_cruise.prb",
        table_files,
        output_dir=artifact_dir / "runtime",
        max_steps=100,
    )
    assert report.artifacts

    output_root = artifact_dir / "generic-cruise-plots"
    generated_plots: list[Path] = []
    for artifact in report.artifacts:
        for series in build_run_plot_series(artifact):
            output_path = output_root / series.filename
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_bytes(
                render_table_png(
                    prepare_table((series.axis,), series.values),
                    axis_labels=(series.axis_label,),
                    value_label=series.value_label,
                    title=series.title,
                )
            )
            generated_plots.append(output_path)
    assert generated_plots
    assert all(path.read_bytes().startswith(b"\x89PNG") for path in generated_plots)
    ####
