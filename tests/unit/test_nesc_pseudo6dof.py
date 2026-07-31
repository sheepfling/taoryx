"""Tests for the NESC source-translation pseudo-6DOF bridge."""

from __future__ import annotations

from taoryx.trajectory import build_nesc_composite_pseudo6dof


def test_nesc_composite_preserves_staging_and_marks_surrogate_boundary() -> None:
    result = build_nesc_composite_pseudo6dof()

    assert result.passed
    assert len(result.rows) == 202
    assert result.rows[0]["phase"] == "stage1_burn"
    assert result.rows[-1]["phase"] == "orbit_coast"
    assert result.rows[0]["translation_source_replay"] is True
    assert result.rows[0]["attitude_source"] == "scheduled_response_surrogate"
    assert result.rows[0]["physical_gimbal_allocation"] is False
    assert all(row["response_profile_id"] == "nesc_rocket.attitude_response_p6dof.v1" for row in result.rows)
    ####


def test_nesc_composite_records_phase_scheduled_response_provenance() -> None:
    """Staged response assumptions remain visible beside source history."""

    result = build_nesc_composite_pseudo6dof()

    assert {str(row["response_phase"]) for row in result.rows} == {"stage1_burn", "stage2_burn", "stack_coast", "orbit_coast"}
    assert all(row["response_schedule_applied"] is True for row in result.rows)
    ####
