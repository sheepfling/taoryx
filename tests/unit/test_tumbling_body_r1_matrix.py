"""Tests for the passive tumbling-body matrix wrapper."""

from __future__ import annotations

from pathlib import Path

from tools.validate_tumbling_body_r1_matrix import FIDELITIES, run_matrix


def test_tumbling_body_matrix_is_passive_and_complete(tmp_path: Path) -> None:
    """The wrapper indexes both interfaces without inventing control."""

    root = Path(__file__).resolve().parents[2]
    report = run_matrix(root / "verification/alpha3_tumbling_body/qualification.json", tmp_path)
    assert report["schema"] == "taoryx.tumbling-body-passive-r1-matrix/v1alpha1"
    assert report["status"] == "PASSIVE_R1_MATRIX_COMPLETE"
    assert report["case_count"] == 4 * len(FIDELITIES)
    assert report["passed_case_count"] == report["case_count"]
    assert "no prescribed tumble" in report["claim_boundary"]
    ####
