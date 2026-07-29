"""Promotion-gate checks for the local F-16 surface path."""

from __future__ import annotations

from tools.validate_f16_racetrack_promotion import validate


def test_f16_local_t5_promotion_requires_all_evidence_layers() -> None:
    """The local T5 report is a conjunction, not a route score."""

    report = validate()
    assert report["status"] == "development_t5_local_pass"
    assert report["qualification_tier"] == "T5_local_nonlinear_validation"
    assert all(report["checks"].values())
    assert report["robustness"]["status"] == "R1_fixed_matrix_boundary_failure_recorded"
    assert "continuous gain scheduling between operating points" in report["nonclaims"]
    ####
