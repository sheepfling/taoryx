from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.fixtures.spectre_trajectories import load_spec, render_workspace

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "problem_file_dumps" / "spectre_trajectories"
SPEC = ROOT / "spec.yaml"
pytestmark = pytest.mark.spectre


def test_spectre_trajectory_spec_renders_the_checked_in_problem_files() -> None:
    spec = load_spec(SPEC)
    rendered = render_workspace(spec)

    assert set(rendered) == {path.name for path in ROOT.glob("*.prb")}
    for name, text in rendered.items():
        assert (ROOT / name).read_text(encoding="utf-8") == text
    ####


def test_spectre_trajectory_manifest_matches_problem_files() -> None:
    manifest = yaml.safe_load((ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = manifest["files"]
    actual = sorted(path.name for path in ROOT.glob("*.prb"))

    assert listed == actual
    assert manifest["workspace"] == "problem_file_dumps"
    assert manifest["leaf"] == "spectre_trajectories"
    assert manifest["generated_spec"] == "spec.yaml"
####


def test_spectre_trajectory_problem_files_are_minimally_structured() -> None:
    for path in sorted(ROOT.glob("*.prb")):
        text = path.read_text(encoding="utf-8").splitlines()
        nonblank = [line for line in text if line.strip()]
        assert nonblank[0].startswith("(")
        assert nonblank[-1] == "*end"
        assert any(line.startswith("# Synthetic TAOS trajectory translation") for line in nonblank)
####
