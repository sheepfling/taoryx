"""Tests for the Hummingbird native/pseudo channel comparison."""

from __future__ import annotations

from pathlib import Path

from tools.validate_hummingbird_native_pseudo_comparison import build_comparison


def test_hummingbird_native_pseudo_comparison_keeps_battery_boundary(tmp_path: Path) -> None:
    report = build_comparison(tmp_path)
    assert report["status"] == "comparison_pass_with_resource_boundary"
    assert report["objective_status_comparison"]["all_mapped_statuses_agree"] is True
    assert report["hover_thrust_comparison"]["status"] == "pass"
    assert report["contact_comparison"]["status"] == "pass"
    assert report["resource_comparison"]["pseudo_battery_monotone_nonincreasing"] is True
    assert report["resource_comparison"]["native_electrical_battery_model_available"] is False
    assert (tmp_path / "comparison.json").is_file()
    assert (tmp_path / "native_pseudo_comparison_board.png").is_file()
    ####
