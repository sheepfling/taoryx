"""Regression checks for the HL-20 source-load allocation witness."""

from __future__ import annotations

from pathlib import Path

from tools.validate_hl20_source_allocation_probe import build_probe


def test_hl20_source_allocation_is_bounded_and_fail_closed(tmp_path: Path) -> None:
    report = build_probe(tmp_path)
    assert report["status"] == "open_loop_source_allocation_pass_with_boundary"
    assert report["summary"]["minimum_matrix_rank"] == 6
    assert report["summary"]["stress_boundary_count"] == 4
    assert report["summary"]["all_feasible_solver_requests_succeeded"] is True
    assert all(
        point["allocator"]["implementation"] == "taoryx.control_allocation.allocate_and_advance_wrench"
        for point in report["operating_points"]
    )
    assert all(
        point["infeasible_request"]["allocation_status"] == "partially_achievable"
        for point in report["operating_points"]
    )
    assert (tmp_path / "source_allocation_board.png").is_file()
####
