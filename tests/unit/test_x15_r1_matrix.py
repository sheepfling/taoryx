"""Tests for the X-15 staged-surrogate R1 matrix."""

from __future__ import annotations

from tools.validate_x15_r1_matrix import CASES, FIDELITIES, run_matrix


def test_x15_r1_matrix_retains_deployment_boundary(tmp_path) -> None:
    """Every reduced witness keeps the explicit deployment/event contract."""

    report = run_matrix(tmp_path)
    assert report["schema"] == "taoryx.x15-r1-matrix/v1alpha1"
    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == len(CASES) * len(FIDELITIES)
    assert report["passed_case_count"] == report["case_count"]
    assert "controlled terminal handoff" in report["claim_boundary"]
    ####
