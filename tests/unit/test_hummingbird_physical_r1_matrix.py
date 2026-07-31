"""Tests for the generated Hummingbird physical R1 contract."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_hummingbird_physical_r1_matrix_retains_rotor_authority_boundary() -> None:
    """Individual-rotor interior cases pass and the boundary remains failed."""

    payload = json.loads(
        (ROOT / "verification/alpha3_hummingbird_physical_r1/manifest.json").read_text(encoding="utf-8")
    )
    assert payload["status"] == "R1_physical_individual_rotor_matrix_complete"
    assert payload["direct_body_moment_injection"] is False
    assert payload["passed_case_count"] == 3
    assert payload["boundary_failure_count"] == 1
    boundary = next(case for case in payload["cases"] if case["boundary_witness"])
    assert boundary["mission_pass"] is False
    assert "infeasible" in boundary["metrics"]["allocation_statuses"]
    ####
