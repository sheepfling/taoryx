"""Tests for the source-effector F-16 schedule-node evidence."""

from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_f16_physical_schedule_nodes_are_complete_and_bounded() -> None:
    """The generated schedule packet records four passing physical nodes."""

    artifact = json.loads(
        (ROOT / "verification/alpha3_f16_physical_schedule/manifest.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "F16_physical_effector_schedule_nodes_complete"
    assert artifact["controller_profile"] == "state_and_wrench_balanced_q10_r0p01"
    assert artifact["direct_body_moment_injection"] is False
    assert artifact["schedule_contract"]["runtime_gain_interpolation"] == "generic_command_contract_exercised"
    summary = artifact["summary"]
    assert summary["case_count"] == 20
    assert summary["failed_case_count"] == 0
    assert summary["minimum_effectiveness_rank"] == 4
    assert summary["node_count"] == 4
    assert summary["passed_case_count"] == 20
    assert summary["passed_node_count"] == 4
    assert summary["transition_probe_count"] == 15
    assert summary["transition_probe_passed_count"] == 15
    assert summary["transition_probe_failed_count"] == 0
    assert all(node["passed_case_count"] == 5 for node in artifact["nodes"])
    assert all(node["failed_case_count"] == 0 for node in artifact["nodes"])
    assert all(node["derivative_consistent"] for node in artifact["nodes"])
    assert all(transition["transition_evidence"] == "generic_interpolated_wrench_command" for transition in artifact["transitions"])
    assert all(transition["midpoint_command_finite"] for transition in artifact["transitions"])
    assert all(
        item["failed_probe_count"] == 0
        for item in artifact["transition_allocation_probes"]
    )
    ####
