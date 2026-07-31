"""Tests for the Alpha 3 readiness matrix."""

from __future__ import annotations

from tools.build_alpha3_readiness_report import build_report


def test_alpha3_readiness_matrix_is_fail_closed_about_r1() -> None:
    report = build_report()
    assert report["status"] == "development_readiness_report"
    assert report["summary"]["family_count"] == 9
    # The passive tumbling body is nominal for its declared averaged-area /
    # native-rigid-body contract, while controller promotion remains blocked.
    assert report["summary"]["nominal_pair_ready_count"] == 9
    assert report["summary"]["passive_pair_ready_count"] == 1
    assert report["summary"]["r1_boundary_witness_count"] == 1
    assert report["summary"]["r1_matrix_complete_count"] == 9
    assert report["summary"]["r1_pending_count"] == 0
    assert report["summary"]["reduced_r1_complete_count"] == 9
    assert report["summary"]["physical_effector_witness_count"] == 4
    assert report["summary"]["physical_effector_t5_count"] == 2
    assert report["summary"]["physical_r1_matrix_count"] == 4
    assert report["summary"]["physical_schedule_envelope_count"] == 1
    assert report["summary"]["directional_translation_witness_count"] == 1
    assert report["summary"]["native_horizontal_witness_count"] == 1
    assert report["summary"]["native_vertical_witness_count"] == 1
    assert report["summary"]["native_pad_to_pad_witness_count"] == 1
    families = {item["family_id"]: item for item in report["families"]}
    assert families["f16_s119"]["r1_fixed_matrix"]["status"] == "boundary_witness_available"
    assert families["skywalker_x8"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["b747"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["b747"]["physical_r1"]["status"] == "R1_physical_surface_matrix_complete"
    assert families["b747"]["physical_r1"]["boundary_failure_count"] == 1
    assert families["b747"]["physical_schedule"]["status"] == "B747_physical_effector_schedule_boundary_recorded"
    assert families["b747"]["physical_schedule"]["passed_node_count"] == 1
    assert families["b747"]["physical_schedule"]["runtime_gain_interpolation"] == "not_run_until_all_source_nodes_complete"
    assert families["b747"]["physical_schedule"]["blockers_by_code"] == {"source_trim_residual_above_tolerance": 3}
    assert {item["point_id"] for item in families["b747"]["physical_schedule"]["blocked_nodes"]} == {"8", "9", "10"}
    assert {item["worst_residual_name"] for item in families["b747"]["physical_schedule"]["blocked_nodes"]} == {"body_x_force_n"}
    b747_boundary_nodes = families["b747"]["physical_schedule"]["boundary_nodes"]
    assert {item["point_id"] for item in b747_boundary_nodes} == {"4", "5", "6", "7"}
    assert all(item["controller_trial_count"] >= 3 for item in b747_boundary_nodes)
    assert all(item["interior_passed"] is False for item in b747_boundary_nodes)
    assert families["f16_s119"]["physical_r1"]["status"] == "R1_physical_effector_matrix_complete"
    assert families["f16_s119"]["physical_r1"]["boundary_failure_count"] == 0
    assert families["f16_s119"]["physical_schedule"]["status"] == "F16_physical_effector_schedule_nodes_complete"
    assert families["f16_s119"]["physical_schedule_transition"]["status"] == "F16_physical_effector_schedule_transition_complete"
    assert families["f16_s119"]["physical_schedule_transition"]["passed_case_count"] == 4
    assert families["f16_s119"]["physical_schedule_envelope"]["status"] == "F16_physical_effector_schedule_envelope_boundary_recorded"
    assert families["f16_s119"]["physical_schedule_envelope"]["case_count"] == 32
    assert families["f16_s119"]["physical_schedule_envelope"]["boundary_case_count"] == 12
    assert families["f16_s119"]["physical_schedule_interior"]["status"] == "F16_physical_schedule_interior_qualified"
    assert families["f16_s119"]["physical_schedule_interior"]["interior_case_count"] == 8
    assert families["f16_s119"]["physical_schedule_interior"]["boundary_case_count"] == 24
    assert "broaden" in families["f16_s119"]["next_gate"]
    assert families["hummingbird"]["physical_r1"]["status"] == "R1_physical_individual_rotor_matrix_complete"
    assert families["hummingbird"]["physical_r1"]["boundary_failure_count"] == 1
    assert families["skywalker_x8"]["physical_r1"]["status"] == "R1_physical_source_coordinate_matrix_complete"
    assert families["skywalker_x8"]["physical_r1"]["boundary_failure_count"] == 1
    assert families["skywalker_x8"]["physical_effector"]["evidence_tier"] == "T4_physically_allocated_source_coordinate"
    assert families["skywalker_x8"]["physical_effector"]["promotion_blocker"] == "individual_left_right_actuator_and_end_to_end_racetrack_evidence_pending"
    assert families["skywalker_x8"]["physical_mapping"]["status"] == "X8_physical_left_right_mapping_resolved_by_source_equation"
    assert families["skywalker_x8"]["physical_mapping"]["promotion_authorized"] is True
    assert families["b747"]["physical_effector"]["evidence_tier"] == "T5_nonlinearly_validated"
    assert families["a320"]["reduced_r1"]["status"] == "complete"
    assert families["f16_s119"]["reduced_r1"]["status"] == "complete"
    assert families["hl20_mod_k"]["reduced_r1"]["status"] == "complete"
    assert families["hl20_mod_k"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["hummingbird"]["reduced_r1"]["status"] == "complete"
    assert families["hummingbird"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["hummingbird"]["directional_translation"]["status"] == "directional_translation_witness_pass"
    assert families["hummingbird"]["directional_translation"]["passed_objective_count"] == 10
    assert families["hummingbird"]["native_horizontal"]["status"] == "native_horizontal_translation_witness_pass_with_authority_boundary"
    assert families["hummingbird"]["native_horizontal"]["mission_pass"] is True
    assert families["hummingbird"]["native_horizontal"]["allocation_summary"]["authority_boundary"] is True
    assert families["hummingbird"]["native_vertical"]["status"] == "native_vertical_force_witness_pass"
    assert families["hummingbird"]["native_vertical"]["mission_pass"] is True
    assert families["hummingbird"]["native_pad_to_pad"]["status"] == "nominal_case_pass_overall_qualification_pending"
    assert families["hummingbird"]["native_pad_to_pad"]["mission_pass"] is True
    assert families["hummingbird"]["native_pad_to_pad"]["passed_objective_count"] == 13
    assert families["hummingbird"]["nominal_pair_ready"] is True
    assert families["hummingbird"]["automatic_lowering_authorized"] is True
    assert families["x15"]["reduced_r1"]["status"] == "complete"
    assert families["x15"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["x15"]["nominal_pair_ready"] is True
    assert families["x15"]["automatic_lowering_authorized"] is True
    assert families["reference_nesc_two_stage_rocket"]["r1_fixed_matrix"]["status"] == "paired_reduced_r1_complete"
    assert families["tumbling_body"]["reduced_r1"]["status"] == "complete"
    assert families["tumbling_body"]["r1_fixed_matrix"]["status"] == "passive_reduced_r1_complete"
    assert families["tumbling_body"]["passive_pair_ready"] is True
    assert families["tumbling_body"]["automatic_lowering_authorized"] is False
    assert families["skywalker_x8"]["automatic_lowering_authorized"] is True
    assert families["hl20_mod_k"]["nominal_pair_ready"] is True
    assert families["hl20_mod_k"]["automatic_lowering_authorized"] is True
    assert families["reference_nesc_two_stage_rocket"]["nominal_pair_ready"] is True
    assert families["reference_nesc_two_stage_rocket"]["automatic_lowering_authorized"] is True
    ####
