from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from taoryx.fixtures.simple_aero_trajectories import load_spec, render_workspace

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "problem_file_dumps" / "simple_aero_trajectories"
SPEC = ROOT / "spec.yaml"
pytestmark = pytest.mark.simple_aero


def test_simple_aero_trajectory_spec_renders_the_checked_in_problem_files() -> None:
    spec = load_spec(SPEC)
    rendered = render_workspace(spec)

    assert set(rendered) == {path.name for path in ROOT.glob("*.prb")}
    for name, text in rendered.items():
        assert (ROOT / name).read_text(encoding="utf-8") == text
    ####


def test_simple_aero_trajectory_manifest_matches_problem_files() -> None:
    manifest = yaml.safe_load((ROOT / "manifest.yaml").read_text(encoding="utf-8"))
    listed = manifest["files"]
    actual = sorted(path.name for path in ROOT.glob("*.prb"))

    assert listed == actual
    assert manifest["workspace"] == "problem_file_dumps"
    assert manifest["leaf"] == "simple_aero_trajectories"
    assert manifest["generated_spec"] == "spec.yaml"
####


def test_simple_aero_trajectory_problem_files_are_minimally_structured() -> None:
    for path in sorted(ROOT.glob("*.prb")):
        text = path.read_text(encoding="utf-8").splitlines()
        nonblank = [line for line in text if line.strip()]
        assert nonblank[0].startswith("(")
        assert nonblank[-1] == "*end"
        assert any(line.startswith("# Synthetic TAOS trajectory translation") for line in nonblank)
####


def test_simple_aero_trajectory_metadata_is_structured() -> None:
    spec = load_spec(SPEC)
    by_name = {Path(problem.path).name: problem.simple_aero_metadata for problem in spec.problems}

    assert by_name["ballistic.prb"].family == "Ballistic"
    assert by_name["cbcr_left.prb"].family == "CBCR"
    assert by_name["cbcr_left.prb"].direction == "left"
    assert by_name["cbcr_right.prb"].direction == "right"
    assert by_name["crossrange.prb"].initial_heading_error_deg == 15.0
    assert by_name["marv.prb"].maneuver_begin_time_to_go_s == 60.0
    assert by_name["phugoid.prb"].phugoid_frequency_Hz == 0.015
    assert by_name["skip.prb"].maneuver_begin_time_to_go_s == 400.0
    assert by_name["propnav.prb"].terminal_handoff_range_km == 20.0
####
