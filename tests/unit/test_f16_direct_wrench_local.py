"""Contract checks for the F-16 direct-wrench local witness."""

from __future__ import annotations

from tools.validate_f16_direct_wrench_local import build_artifact


def test_f16_direct_wrench_local_witness_is_explicit() -> None:
    artifact = build_artifact()
    assert artifact["status"] == "nominal_case_pass"
    assert artifact["fidelity"] == "rigid_body_6dof_direct_wrench"
    assert artifact["claim"]["direct_body_moment_injection"] is True
    assert artifact["claim"]["physical_effector_allocation"] is False
    assert artifact["metrics"]["final_feedback_error_fraction"] < 0.20
    ####
