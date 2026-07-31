"""Regression checks for the X8 direct-wrench 6DOF witness."""

from __future__ import annotations

from tools.validate_x8_direct_wrench import build_artifact


def test_x8_direct_wrench_witness_passes_with_explicit_boundary() -> None:
    """The local X8 direct-wrench path closes without physical-effector claims."""

    artifact = build_artifact()
    assert artifact["evaluation"]["mission_pass"] is True
    assert artifact["fidelity"] == "rigid_body_6dof_direct_wrench"
    assert artifact["claim"]["direct_body_moment_injection"] is True
    assert artifact["claim"]["physical_effector_allocation"] is False
    assert artifact["linearization"]["maximum_real_pole"] < 0.0
    assert "partially_achievable" in artifact["evaluation"]["statuses_observed"]
    ####
