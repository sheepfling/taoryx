"""Tests for the HL-20 source-control direction witness."""

from __future__ import annotations

from pathlib import Path

from tools.validate_hl20_source_control_probe import build_probe


def test_hl20_source_control_probe_is_open_loop_and_complete(tmp_path: Path) -> None:
    report = build_probe(tmp_path)
    assert report["status"] == "open_loop_source_direction_pass_with_boundary"
    assert len(report["operating_points"]) == 4
    assert all(point["nonzero_surface_count"] >= 6 for point in report["operating_points"])
    assert all(point["response_matrix_rank"] >= 2 for point in report["operating_points"])
    assert (tmp_path / "control_direction_probe.json").is_file()
    assert (tmp_path / "source_control_direction_board.png").is_file()
    ####
