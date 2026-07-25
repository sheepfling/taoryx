from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.fixtures.simple_aero_trajectories import load_spec
from taoryx.fixtures.simple_aero_trajectory_artifacts import build_problem_plot_series
from taoryx.tables import prepare_table
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "problem_file_dumps" / "simple_aero_trajectories"
SPEC = ROOT / "spec.yaml"

pytestmark = [pytest.mark.simple_aero, pytest.mark.artifact]


def test_simple_aero_trajectory_plot_generator_uses_expected_labels_and_series() -> None:
    spec = load_spec(SPEC)
    ballistic = next(problem for problem in spec.problems if Path(problem.path).name == "ballistic.prb")
    propnav = next(problem for problem in spec.problems if Path(problem.path).name == "propnav.prb")

    ballistic_series = build_problem_plot_series(ballistic)
    propnav_series = build_problem_plot_series(propnav)

    assert {series.filename for series in ballistic_series} >= {
        "trajectory-1-range-altitude.png",
        "trajectory-1-speed-time.png",
        "trajectory-1-mass-time.png",
        "trajectory-1-x-time.png",
        "trajectory-1-y-time.png",
        "trajectory-1-closing-range-time.png",
    }
    assert {series.value_label for series in ballistic_series} >= {
        "altitude (m)",
        "speed (m/s)",
        "mass (kg)",
        "x (m)",
        "y (m)",
        "range (m)",
    }
    assert {series.filename for series in propnav_series} >= {
        "trajectory-1-range-altitude.png",
        "trajectory-1-speed-time.png",
        "trajectory-1-mass-time.png",
        "trajectory-1-x-time.png",
        "trajectory-1-y-time.png",
        "trajectory-1-bank-time.png",
        "trajectory-1-alpha-time.png",
        "trajectory-1-gamma-time.png",
        "trajectory-1-thrust-time.png",
        "trajectory-2-range-altitude.png",
        "trajectory-2-closing-range-time.png",
    }
    assert {series.value_label for series in propnav_series} >= {
        "altitude (m)",
        "speed (m/s)",
        "mass (kg)",
        "x (m)",
        "y (m)",
        "bank angle (deg)",
        "alpha (deg)",
        "gamma (deg)",
        "thrust (arb)",
        "range (m)",
    }
####


def test_simple_aero_trajectory_workspace_renders_png_artifacts(artifact_dir: Path) -> None:
    spec = load_spec(SPEC)
    output_root = artifact_dir / "simple_aero-trajectories-plots"
    generated: list[Path] = []

    for problem in spec.problems:
        problem_root = output_root / Path(problem.path).with_suffix("")
        for series in build_problem_plot_series(problem):
            output_path = problem_root / series.filename
            output_path.parent.mkdir(parents=True, exist_ok=True)
            table = prepare_table((series.axis,), series.values)
            output_path.write_bytes(
                render_table_png(
                    table,
                    axis_labels=(series.axis_label,),
                    value_label=series.value_label,
                    title=series.title,
                )
            )
            generated.append(output_path)
    ####

    index = output_root / "manifest.txt"
    index.write_text(
        "\n".join(str(path.relative_to(output_root)) for path in sorted(generated)),
        encoding="utf-8",
    )

    assert generated
    assert all(path.exists() for path in generated)
    assert index.exists()
####
