"""Regression checks for the fresh-process DAVE-ML release gate."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_daveml_release_gate_is_verified_and_hashes_artifacts() -> None:
    report = json.loads((ROOT / "verification/daveml_release_gate.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert all(item["returncode"] == 0 for item in report["runs"])
    assert all(len(value) == 64 for value in report["artifacts"].values())
    commands = {tuple(item["command"]) for item in report["runs"]}
    assert any(command[-1] == "tools/validate_daveml_alpha3_qualification.py" for command in commands)
    required_artifacts = {
        "verification/daveml_alpha3_qualification.json",
        "verification/daveml_f16_overlay_qualification.json",
        "verification/daveml_f16_reduction_qualification.json",
        "verification/daveml_nesc_staging_lineage.json",
        "verification/daveml_nesc_reduction_qualification.json",
        "verification/daveml_a320_matched_comparison.json",
    }
    assert required_artifacts <= set(report["artifacts"])
