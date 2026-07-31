"""Contract checks for the A320 direct-wrench evidence witness."""

from __future__ import annotations

from tools.validate_a320_direct_wrench import build_artifact


def test_a320_direct_wrench_witness_is_bounded_and_explicit() -> None:
    artifact = build_artifact()
    assert artifact["status"] == "nominal_case_pass"
    assert artifact["fidelity"] == "rigid_body_6dof_direct_wrench"
    claim = artifact["claim"]
    assert claim["direct_body_moment_injection"] is True
    assert claim["physical_effector_allocation"] is False
    assert artifact["metrics"]["final_feedback_error_fraction"] < 0.25
    assert "partially_achievable" in artifact["metrics"]["wrench_statuses_observed"]
    ####
