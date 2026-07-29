"""Regression checks for the HL-20 glide-trim scenario evidence."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_hl20_scenario_is_verified_and_source_linked() -> None:
    report = json.loads((ROOT / "verification/daveml_hl20_scenario_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "pass"
    assert report["objective_report"]["score"] >= 99.99
    assert len(report["provenance"]["aerodynamics_package_sha256"]) == 64
