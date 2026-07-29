"""Regression checks for the DAVE-ML full source-channel tuning artifact."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_f16_tuning_evidence_is_full_state_controllable_and_bounded() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_tuning_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["controllable"] is True
    assert report["hurwitz"] is True
    assert report["excluded_states"] == {}
    assert len(report["gain"]) == 4
    assert all(len(row) == 6 for row in report["gain"])
    assert not report["bounded_probe"]["saturated"]
    assert all(len(value) == 64 for value in report["provenance"].values())
