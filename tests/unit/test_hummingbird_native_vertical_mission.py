"""Tests for the native Hummingbird physical vertical-force witness."""

from __future__ import annotations

from tools.validate_hummingbird_native_vertical_mission import (
    evaluate_native_mission,
    run_native_mission,
)


def test_native_vertical_witness_passes_through_physical_collective_force() -> None:
    rows, _, metadata = run_native_mission(dt_s=0.02)
    evaluation = evaluate_native_mission(rows, dt_s=0.02)

    assert metadata["direct_body_force_injection"] is False
    assert metadata["direct_body_moment_injection"] is False
    assert evaluation["independent_truth_evaluation"] is True
    assert evaluation["mission_pass"] is True
    assert all(item["truth_result"] == "PASS" for item in evaluation["required_objectives"])
    assert max(float(row["achieved_wrench"]["force_z_n"]) for row in rows) > 0.0
    assert min(float(row["achieved_wrench"]["force_z_n"]) for row in rows) < 0.0
    allocation = evaluation["allocation_summary"]
    assert allocation["allocation_pass"] is True
    assert allocation["disallowed_statuses"] == []
    ####


def test_native_vertical_witness_does_not_self_certify_a_missed_gate() -> None:
    rows, _, _ = run_native_mission(dt_s=0.02)
    for row in rows:
        if row["phase_id"] == "vertical_return_hover":
            row["down_position_m"] = 100.0
            row["vertical_speed_down_m_s"] = 2.0
    evaluation = evaluate_native_mission(rows, dt_s=0.02)

    assert evaluation["mission_pass"] is False
    assert evaluation["required_objectives"][-1]["truth_result"] == "FAIL"
    ####
