"""Regression checks for DAVE-ML local dynamics linearization evidence."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_linearization_evidence_has_true_dynamics_shape_and_provenance() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_linearization_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["claim_boundary"].startswith("source-channel operating point")
    assert len(report["a_matrix"]) == 6
    assert all(len(row) == 6 for row in report["a_matrix"])
    assert len(report["b_matrix"]) == 6
    assert all(len(row) == 3 for row in report["b_matrix"])
    assert report["b_matrix"][3][0] != 0.0 or report["b_matrix"][4][0] != 0.0 or report["b_matrix"][5][0] != 0.0
    assert all(len(value) == 64 for value in report["provenance"].values())
