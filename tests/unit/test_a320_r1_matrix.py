"""Tests for the generated A320 reduced-tier R1 matrix."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_a320_r1_matrix_covers_both_reduced_tiers() -> None:
    payload = json.loads((ROOT / "verification/alpha3_a320_r1/manifest.json").read_text(encoding="utf-8"))
    assert payload["schema"] == "taoryx.a320-r1-matrix/v1alpha1"
    assert payload["status"] == "R1_fixed_matrix_complete"
    assert payload["case_count"] == 12
    assert payload["passed_case_count"] == 12
    assert payload["failed_case_count"] == 0
    assert {record["mode"] for record in payload["cases"]} == {
        "point_mass_3dof",
        "pseudo_6dof_kinematic_bridge",
    }
    ####
