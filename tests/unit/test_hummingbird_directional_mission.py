"""Tests for the Hummingbird directional-translation witness."""

from __future__ import annotations

import json

from tools.validate_hummingbird_directional_mission import (
    evaluate_directional,
    run_directional_mission,
    write_directional_packet,
)


def test_directional_mission_passes_both_declared_reduced_tiers() -> None:
    pseudo_rows, phases = run_directional_mission()
    assert len(phases) == 10
    evaluation = evaluate_directional(pseudo_rows)
    assert evaluation["mission_pass"] is True
    results = {item["id"]: item for item in evaluation["required_objectives"]}
    assert all(item["truth_result"] == "PASS" for item in results.values())
    assert results["forward_body_leg"]["actual"]["sustained_duration_s"] >= 1.0
    assert results["lateral_body_right_leg"]["actual"]["peak_signed_body_speed_m_s"] > 0.5
    assert results["rearward_body_leg"]["actual"]["peak_signed_body_speed_m_s"] > 0.5
    ####


def test_directional_evaluator_rejects_a_missed_rearward_leg() -> None:
    rows, _ = run_directional_mission()
    missed = [dict(row) for row in rows]
    for row in missed:
        if row["phase_id"] == "rearward_body_leg":
            row["position_m"] = [8.0, 0.0, 3.0]
            row["velocity_m_s"] = [0.0, 0.0, 0.0]
            row["body_velocity_m_s"] = [0.0, 0.0, 0.0]
    evaluation = evaluate_directional(missed)
    assert evaluation["mission_pass"] is False
    results = {item["id"]: item["truth_result"] for item in evaluation["required_objectives"]}
    assert results["rearward_body_leg"] == "FAIL"
    ####


def test_directional_packet_declares_point_mass_yaw_boundary(tmp_path) -> None:
    manifest = write_directional_packet(tmp_path)
    assert manifest["status"] == "directional_translation_witness_pass"
    assert manifest["physical_motor_allocation"] is False
    point = json.loads((tmp_path / "point_mass_3dof_evidence.json").read_text(encoding="utf-8"))
    results = {item["id"]: item for item in point["evaluation"]["required_objectives"]}
    assert results["yaw_scan_gate"]["truth_result"] == "NOT_APPLICABLE"
    assert point["evaluation"]["mission_pass"] is True
    ####
