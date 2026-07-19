from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from taoryx.contracts import Vector3
from taoryx.language import GrammarProfile, parse_problem_file
from taoryx.runtime.lowering import _limit_body_direction_sideslip
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
MISSION = ROOT / "examples/showcases/x15_rocket_to_hawaii/mission.prb"
X15_TABLE = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl"

pytestmark = [pytest.mark.slow, pytest.mark.artifact, pytest.mark.spectre]


def test_sideslip_command_projection_preserves_pitch_plane() -> None:
    direction = _limit_body_direction_sideslip(Vector3(1.0, 1.0, 0.5), 0.1)

    assert direction.norm() == pytest.approx(1.0)
    assert abs(direction.y) == pytest.approx(math.sin(0.1))
    assert direction.x > 0.0
    assert direction.z > 0.0
####


def test_x15_showcase_declares_taoryx_extension_boundary() -> None:
    legacy = parse_problem_file(MISSION, profile=GrammarProfile.TAOS96)
    successor = parse_problem_file(MISSION, profile=GrammarProfile.TAORYX)

    assert any(item.code == "taoryx-extension-requires-profile" for item in legacy.diagnostics)
    assert not [item for item in successor.diagnostics if item.severity.value == "error"]
####


def test_x15_rocket_release_is_native_and_records_hawaii_attempt(tmp_path: Path) -> None:
    report = run_files(MISSION, (X15_TABLE,), output_dir=tmp_path / "run", max_steps=12000, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 2
    assert not report.results
    assert any(item.code == "runtime-execution-failed" for item in report.diagnostics)
    assert any("outside its declared envelope" in item.message for item in report.diagnostics)
####


@pytest.mark.artifact
def test_x15_rocket_release_renders_standard_run_artifact(tmp_path: Path, artifact_dir: Path) -> None:
    report = run_files(MISSION, (X15_TABLE,), output_dir=tmp_path / "run", max_steps=12000, profile=GrammarProfile.TAORYX)

    assert report.exit_code == 2
    output = artifact_dir / "x15-rocket-to-hawaii" / "run-evidence.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report.as_dict(), indent=2), encoding="utf-8")
    assert output.exists()
    assert "outside its declared envelope" in output.read_text(encoding="utf-8")
####
