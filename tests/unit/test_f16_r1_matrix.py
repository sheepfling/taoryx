"""Tests for the F-16 reduced-tier R1 matrix."""

from __future__ import annotations

from tools.validate_f16_r1_matrix import CASES, MODES, run_matrix


def test_f16_reduced_r1_matrix_preserves_boundary_cases(tmp_path) -> None:
    """Both reduced tiers execute every declared case and retain failures."""

    report = run_matrix(tmp_path)
    assert report["schema"] == "taoryx.f16-r1-matrix/v1alpha1"
    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == len(CASES) * len(MODES)
    assert report["passed_case_count"] < report["case_count"]
    assert report["failed_case_count"] > 0
    assert {case["mode"] for case in report["cases"]} == set(MODES)
    assert {case["case_id"] for case in report["cases"]} == set(CASES)
    assert all(case["numerical_valid"] for case in report["cases"])
    ####
