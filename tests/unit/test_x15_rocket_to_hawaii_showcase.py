from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import run_files
from taoryx.visualization import render_run_artifact_html

ROOT = Path(__file__).resolve().parents[2]
MISSION = ROOT / "examples/showcases/x15_rocket_to_hawaii/mission.prb"
X15_TABLE = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl"

pytestmark = [pytest.mark.slow, pytest.mark.artifact, pytest.mark.spectre]


def test_x15_rocket_release_is_native_and_records_hawaii_attempt(tmp_path: Path) -> None:
    report = run_files(MISSION, (X15_TABLE,), output_dir=tmp_path / "run", max_steps=12000, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    history = report.results[0].states["1"]
    assert any(state.named.get("_segment") == pytest.approx(3.0) for state in history)
    release = next(state for state in history if state.named.get("_segment") == pytest.approx(3.0))
    assert release.named["mass_kg"] == pytest.approx(14641.0545, rel=1e-6)
    assert release.named["range_to_target_m"] > 1.0e6
    assert report.metadata[0]["native_pipeline"]["integration_frame"] == "ecic"
    assert report.metadata[0]["native_pipeline"]["table_binding"] == "explicit-segment-aero"
####


@pytest.mark.artifact
def test_x15_rocket_release_renders_standard_run_artifact(tmp_path: Path, artifact_dir: Path) -> None:
    report = run_files(MISSION, (X15_TABLE,), output_dir=tmp_path / "run", max_steps=12000, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 0
    assert report.artifacts
    output = artifact_dir / "x15-rocket-to-hawaii" / "run.html"
    output.parent.mkdir(parents=True, exist_ok=True)
    render_run_artifact_html(
        report.artifacts[0],
        output,
        vehicle_id="1",
        channels=("altitude_m", "longitude_deg", "latitude_deg", "mach", "mass_kg", "range_to_target_m"),
    )
    assert output.exists()
    assert "range_to_target_m" in output.read_text(encoding="utf-8")
####
