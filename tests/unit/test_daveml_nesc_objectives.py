"""Regression checks for NESC source-retained objective evidence."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_nesc_checkpoint_objectives_are_source_bounded_and_passing() -> None:
    report = json.loads((ROOT / "verification/daveml_nesc_objectives_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "pass"
    assert report["objective_report"]["required_passed"] == 4
    assert report["claim_boundary"] == "source-retained NESC Scenario 17 checkpoint objective qualification; not independent trajectory equivalence"
