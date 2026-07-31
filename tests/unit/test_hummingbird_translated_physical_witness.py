"""Regression checks for translated-body Hummingbird physical evidence."""

from __future__ import annotations

from pathlib import Path

from tools.validate_hummingbird_translated_physical_witness import build_witness


def test_hummingbird_translated_body_witness_passes_without_claiming_guidance(tmp_path: Path) -> None:
    report = build_witness(tmp_path)
    assert report["status"] == "translated_body_physical_rotor_witness_pass"
    assert report["summary"]["case_count"] == 5
    assert report["summary"]["passed_case_count"] == 5
    assert any("waypoint completion" in item for item in report["nonclaims"])
####
