from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.fixtures.spectre_trajectories import load_spec
from taoryx.tables import prepare_table
from taoryx.visualization import render_table_png

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "problem_file_dumps" / "spectre_trajectories"
SPEC = ROOT / "spec.yaml"

pytestmark = [pytest.mark.spectre, pytest.mark.artifact]


def _problem_metrics(problem: object) -> tuple[float, ...]:
    trajectories = getattr(problem, "trajectories")
    return (
        float(len(trajectories)),
        float(sum(len(trajectory.segments) for trajectory in trajectories)),
        float(sum(len(segment.body) for trajectory in trajectories for segment in trajectory.segments)),
        float(len(getattr(problem, "comments"))),
        float(len({segment.phase for trajectory in trajectories for segment in trajectory.segments})),
    )
####


def test_spectre_trajectory_workspace_renders_png_artifacts(artifact_dir: Path) -> None:
    spec = load_spec(SPEC)
    output_root = artifact_dir / "spectre-trajectories-plots"
    generated: list[Path] = []

    for problem in spec.problems:
        metrics = _problem_metrics(problem)
        summary_table = prepare_table(((0.0, 1.0, 2.0, 3.0, 4.0),), metrics)
        output_path = output_root / Path(problem.path).with_suffix(".png")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_bytes(
            render_table_png(
                summary_table,
                axis_labels=("summary metric",),
                title=f"{problem.path} | trajectories, segments, body lines, comments, phases",
            )
        )
        generated.append(output_path)

    index = output_root / "manifest.txt"
    index.write_text(
        "\n".join(str(path.relative_to(output_root)) for path in sorted(generated)),
        encoding="utf-8",
    )

    assert generated
    assert all(path.exists() for path in generated)
    assert index.exists()
####
