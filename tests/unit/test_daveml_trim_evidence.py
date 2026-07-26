"""Regression checks for the reproducible DaveML trim evidence tool."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_daveml_trim_evidence_is_source_backed_and_bounded() -> None:
    report = json.loads((ROOT / "verification/daveml_f16_trim_evidence.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified"
    assert report["claim_boundary"] == "source-bounded pitch-channel trim; not full 6-DOF equilibrium"
    assert report["family_id"] == "reference_f16_s119"
    assert len(report["source"]["document_sha256"]) == 64
    assert len(report["source"]["package_sha256"]) == 64
    assert report["trim"]["max_residual"] < 1.0e-9
    assert -24.0 <= report["trim"]["controls"]["elevator_deg"] <= 24.0
    assert report["local_residual_jacobian"]["b"][0][0] < 0.0
