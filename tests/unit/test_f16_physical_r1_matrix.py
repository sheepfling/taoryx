"""Tests for the fail-closed F-16 physical-effector R1 artifact."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_physical_r1_uses_generic_tuning_without_direct_injection() -> None:
    """The tuned local physical path closes its fixed witness matrix honestly."""

    report = json.loads((ROOT / "verification/alpha3_f16_physical_r1/manifest.json").read_text(encoding="utf-8"))
    assert report["status"] == "R1_physical_effector_matrix_complete"
    assert report["direct_body_moment_injection"] is False
    assert report["all_numerically_valid"] is True
    assert report["case_count"] == 5
    assert report["passed_case_count"] == 5
    assert report["boundary_failure_count"] == 0
    assert report["tuning_profile"] == "state_and_wrench_balanced_q10_r0p01"
    assert all(case["metrics"]["mission_pass"] for case in report["cases"])
    ####
