"""Regression checks for HL-20 lifting-body load evidence."""

import json
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.artifact


def test_hl20_load_evidence_is_source_linked_and_sign_consistent() -> None:
    report = json.loads((ROOT / "verification/daveml_hl20_load_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["loads"]["lift_n"] > 0.0
    assert report["loads"]["body_force_z_n"] == -report["loads"]["lift_n"]
    assert report["loads"]["body_force_x_n"] == -report["loads"]["drag_n"]
    assert all(len(value) == 64 for value in report["provenance"].values())
