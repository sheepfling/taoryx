"""Tests for the generated B747 physical-surface R1 contract."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_b747_physical_r1_matrix_retains_interior_and_boundary_evidence() -> None:
    """Interior witnesses pass while the authority boundary remains visible."""

    payload = json.loads(
        (ROOT / "verification/alpha3_b747_physical_r1/manifest.json").read_text(encoding="utf-8")
    )
    assert payload["status"] == "R1_physical_surface_matrix_complete"
    assert payload["direct_body_moment_injection"] is False
    assert payload["passed_case_count"] == 3
    assert payload["boundary_failure_count"] == 1
    assert payload["failed_case_count"] == 1
    boundary = next(case for case in payload["cases"] if case["boundary_witness"])
    assert boundary["mission_pass"] is False
    assert set(boundary["metrics"]["allocation_statuses"]) == {"feasible", "partially_achievable"}
    ####
