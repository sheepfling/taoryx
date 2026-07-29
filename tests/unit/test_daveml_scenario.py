"""Regression checks for DAVE-ML smoke scenario and objective evidence."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_f16_daveml_scenario_scores_required_objectives() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_scenario_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "pass"
    assert report["objective_report"]["required_passed"] == 3
    assert report["scenario_contract"]["family"] == "reference_f16_s119"
    assert len(report["scenario_contract"]["contract_sha256"]) == 64
