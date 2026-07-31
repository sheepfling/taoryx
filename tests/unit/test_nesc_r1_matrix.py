"""Tests for the NESC reduced-tier R1 matrix."""

from __future__ import annotations

from tools.validate_nesc_r1_matrix import CASES, run_matrix


def test_nesc_r1_matrix_is_fail_closed_about_gimbals(tmp_path) -> None:
    """Response-law sensitivity passes without manufacturing gimbal evidence."""

    report = run_matrix(tmp_path)
    assert report["schema"] == "taoryx.nesc-r1-matrix/v1alpha1"
    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == len(CASES) * 2
    assert report["passed_case_count"] == report["case_count"]
    assert report["perturbation_contract"]["physical_gimbal_allocation"] is False
    assert "gimbal effectiveness" in report["claim_boundary"]
    ####
