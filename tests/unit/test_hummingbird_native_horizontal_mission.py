"""Tests for the native Hummingbird physical horizontal witness."""

from __future__ import annotations

from tools.validate_hummingbird_native_horizontal_mission import (
    evaluate_native_mission,
    run_native_mission,
)


def test_native_horizontal_witness_passes_with_explicit_authority_boundary() -> None:
    rows, _, metadata = run_native_mission(dt_s=0.02)
    evaluation = evaluate_native_mission(rows, dt_s=0.02)

    assert metadata["direct_body_moment_injection"] is False
    assert evaluation["independent_truth_evaluation"] is True
    assert evaluation["mission_pass"] is True
    assert all(item["truth_result"] == "PASS" for item in evaluation["required_objectives"])
    allocation = evaluation["allocation_summary"]
    assert allocation["allocation_pass"] is True
    assert allocation["authority_boundary"] is True
    assert allocation["disallowed_statuses"] == []
    ####


def test_native_horizontal_witness_does_not_self_certify_a_missed_objective() -> None:
    rows, _, _ = run_native_mission(dt_s=0.02)
    for row in rows:
        if row["phase_id"] == "horizontal_return_gate":
            row["position_xy_m"] = [100.0, 100.0]
    evaluation = evaluate_native_mission(rows, dt_s=0.02)

    assert evaluation["mission_pass"] is False
    assert evaluation["required_objectives"][-1]["truth_result"] == "FAIL"
    ####
