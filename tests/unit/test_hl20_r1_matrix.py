"""Tests for the HL-20 passive/open-loop reduced-tier R1 matrix."""

from __future__ import annotations

from tools.validate_hl20_r1_matrix import CASES, FIDELITIES, run_matrix


def test_hl20_reduced_r1_matrix_is_explicitly_passive(tmp_path) -> None:
    """All declared release witnesses run without creating controller claims."""

    report = run_matrix(tmp_path)
    assert report["schema"] == "taoryx.hl20-r1-matrix/v1alpha1"
    assert report["status"] == "R1_fixed_matrix_complete"
    assert report["case_count"] == len(CASES) * len(FIDELITIES)
    assert report["passed_case_count"] == report["case_count"]
    assert "closed-loop control" in report["claim_boundary"]
    ####
