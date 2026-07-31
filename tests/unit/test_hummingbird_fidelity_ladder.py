"""Tests for the Hummingbird Alpha 3 truth-evaluated pseudo packet."""

from __future__ import annotations

import json

from tools.validate_hummingbird_fidelity_ladder import _evaluate, run_mission, write_hummingbird_fidelity_ladder


def test_hummingbird_development_mission_passes_independent_objectives() -> None:
    rows, phases = run_mission()
    evaluation = _evaluate(rows)

    assert len(phases) == 9
    assert evaluation["mission_pass"] is True
    assert all(objective["truth_result"] == "PASS" for objective in evaluation["required_objectives"])
    assert evaluation["terminal_pass"] is True
    results = {objective["id"]: objective for objective in evaluation["required_objectives"]}
    assert results["disturbance_recovery"]["truth_result"] == "PASS"
    ####


def test_hummingbird_evaluator_rejects_a_missed_box_corner() -> None:
    rows, _ = run_mission()
    missed = [dict(row) for row in rows]
    for row in missed:
        if row["phase_id"] == "box_north":
            row["position_m"] = [0.0, 0.0, 2.0]
            row["velocity_m_s"] = [0.0, 0.0, 0.0]
    evaluation = _evaluate(missed)

    assert evaluation["mission_pass"] is False
    results = {objective["id"]: objective["truth_result"] for objective in evaluation["required_objectives"]}
    assert results["box_north"] == "FAIL"
    ####


def test_hummingbird_packet_attaches_native_parent_by_semantic_objective_mapping(tmp_path) -> None:
    manifest = write_hummingbird_fidelity_ladder(tmp_path)

    assert manifest["parent_native_mission"] is not None
    parent = manifest["parent_native_mission"]
    assert parent["mission_pass"] is True
    assert parent["all_mapped_objectives_agree"] is True
    assert parent["unmapped_pseudo_objectives"] == ["disturbance_recovery"]
    assert parent["comparison_boundary"].startswith("semantic objective comparison only")
    ####


def test_hummingbird_point_mass_packet_is_independent_and_declares_yaw_unavailable(tmp_path) -> None:
    write_hummingbird_fidelity_ladder(tmp_path)
    point = json.loads((tmp_path / "point_mass_3dof_evidence.json").read_text(encoding="utf-8"))
    assert point["status"] == "nominal_case_pass"
    assert point["control_path"]["realization"] == "force_model"
    assert point["control_path"]["aggregate_thrust_vector_surrogate"] is False
    assert point["control_path"]["physical_motor_allocation"] is False
    assert point["evaluation"]["mission_pass"] is True

    objectives = {item["id"]: item for item in point["evaluation"]["required_objectives"]}
    assert objectives["yaw_gate"]["truth_result"] == "NOT_APPLICABLE"
    assert objectives["yaw_gate"]["required"] is False
    assert all(
        objectives[objective_id]["truth_result"] == "PASS"
        for objective_id in (
            "takeoff_altitude_gate",
            "box_east",
            "box_north",
            "box_west",
            "box_south_return",
            "disturbance_recovery",
            "touchdown_and_post_contact_settle",
        )
    )
    ####
