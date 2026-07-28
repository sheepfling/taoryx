"""Regression checks for the source-backed F-16 equilibrium trim."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_f16_equilibrium_trim_evidence_converges_inside_bounds() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["max_residual"] < 1.0e-8
    assert report["bounds"]["alpha_deg"][0] <= report["state"]["alpha_deg"] <= report["bounds"]["alpha_deg"][1]
    assert report["bounds"]["elevator_deg"][0] <= report["controls"]["elevator_deg"] <= report["bounds"]["elevator_deg"][1]
    assert report["bounds"]["power_pct"][0] <= report["controls"]["power_pct"] <= report["bounds"]["power_pct"][1]
    assert all(len(value) == 64 for value in report["provenance"].values())
