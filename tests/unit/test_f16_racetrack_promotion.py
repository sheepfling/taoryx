"""Promotion-gate checks for the local F-16 surface path."""

from __future__ import annotations

import json
from pathlib import Path

from tools.validate_f16_racetrack_promotion import validate

ROOT = Path(__file__).resolve().parents[2]


def test_f16_local_t5_promotion_requires_all_evidence_layers() -> None:
    """The local T5 report is a conjunction, not a route score."""

    report = validate()
    assert report["status"] == "development_t5_local_pass"
    assert report["qualification_tier"] == "T5_local_nonlinear_validation"
    assert all(report["checks"].values())
    assert report["robustness"]["status"] == "R1_fixed_matrix_boundary_failure_recorded"
    assert "continuous gain scheduling between operating points" in report["nonclaims"]
    ####


def test_f16_reduced_packet_records_semantic_mission_window_parity() -> None:
    """Reduced routes report parity and disagreement without claiming equivalence."""

    artifact = json.loads(
        (
            ROOT
            / "artifacts/showcases/f16-s119-racetrack-fidelity-ladder/reduction_mission_window_comparison.json"
        ).read_text(encoding="utf-8")
    )
    assert artifact["status"] == "semantic_mission_parity_passed"
    for record in artifact["records"].values():
        assert record["objective_parity"] is True
        assert record["mission_status_parity"] is True
        assert record["trajectory_disagreement"]["east_m"]["maximum_absolute"] > 0.0
        assert "no parent-plant or actuator equivalence" in record["claim_boundary"]
    ####
