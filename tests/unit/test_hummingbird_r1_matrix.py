"""Tests for the Hummingbird aggregate-thrust R1 matrix."""

from __future__ import annotations

from tools.validate_hummingbird_r1_matrix import CASES, run_matrix


def test_hummingbird_r1_matrix_keeps_projection_boundary(tmp_path) -> None:
    """The paired records pass while preserving the motor-allocation nonclaim."""

    report = run_matrix(tmp_path)
    assert report["schema"] == "taoryx.hummingbird-r1-matrix/v1alpha1"
    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == len(CASES) * 2
    assert report["passed_case_count"] == report["case_count"]
    assert "individual motor/rotor allocation" in report["claim_boundary"]
    ####
