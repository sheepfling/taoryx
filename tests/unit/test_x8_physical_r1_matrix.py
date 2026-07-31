"""Contract checks for the X8 source-coordinate physical R1 matrix."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_x8_physical_r1_retains_interior_passes_and_beta_boundary() -> None:
    """The local matrix does not convert an uncontrolled beta boundary into a pass."""

    report = json.loads((ROOT / "verification/alpha3_x8_physical_r1/manifest.json").read_text(encoding="utf-8"))
    assert report["status"] == "R1_physical_source_coordinate_matrix_complete"
    assert report["direct_body_moment_injection"] is False
    assert report["physical_mapping"]["status"] == "selected"
    assert report["physical_mapping"]["differential_sign"] == 1
    assert report["passed_case_count"] == 4
    assert report["boundary_failure_count"] == 1
    boundary = next(item for item in report["cases"] if item["boundary_witness"])
    assert boundary["status"] == "boundary"
    assert boundary["failure"]["code"] == "DATA_DOMAIN_VIOLATION"
    assert "beta" in boundary["failure"]["message"]
    assert report["uncontrolled_wrench_axes"] == ["moment_z_nm"]
    ####
