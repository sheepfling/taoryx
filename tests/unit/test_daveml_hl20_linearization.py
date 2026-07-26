"""Regression checks for HL-20 source-trim linearization evidence."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_hl20_linearization_is_source_linked() -> None:
    report = json.loads((ROOT / "verification/daveml_hl20_linearization_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["claim_boundary"].startswith("source-bounded")
    assert len(report["a_matrix"]) == 1
    assert len(report["a_matrix"][0]) == 1
    assert report["b_matrix"] == [[]]
    assert all(len(value) == 64 for value in report["provenance"].values())
