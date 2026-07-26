"""Regression checks for the requirement-level DaveML audit."""

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_completion_audit_preserves_known_gaps() -> None:
    report = json.loads((ROOT / "verification/daveml_completion_audit.json").read_text(encoding="utf-8"))
    assert report["status"] == "verified_with_known_gaps"
    statuses = {item["id"]: item["status"] for item in report["requirements"]}
    assert statuses["fresh_process_release_gate"] == "verified"
    assert statuses["official_2d_ungridded_interpolation"] == "known_gap_quarantined"
    assert statuses["a320_surrogate_composite_6dof_lane"] == "bounded_runtime_verified"
    assert statuses["a320_exact_source_package"] == "blocked_missing_source"
