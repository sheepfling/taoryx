"""Tests for the fail-closed B747 source-effector schedule packet."""

from __future__ import annotations

import json
import math
from pathlib import Path

from taoryx.contracts import Vector3
from tools.validate_b747_physical_schedule import B747NodeIntegrationFailure, _filter, _node_quaternion, _read_conditions

ROOT = Path(__file__).resolve().parents[2]


def test_b747_schedule_records_source_nodes_without_promoting_failed_trim() -> None:
    """The packet separates re-trimmed nodes from unresolved source nodes."""

    artifact = json.loads(
        (ROOT / "verification/alpha3_b747_physical_schedule/manifest.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "B747_physical_effector_schedule_boundary_recorded"
    assert artifact["direct_body_moment_injection"] is False
    assert artifact["summary"]["node_count"] == 8
    assert artifact["summary"]["passed_node_count"] == 1
    assert artifact["summary"]["blocked_node_count"] == 3
    assert artifact["summary"]["boundary_node_count"] == 4
    assert artifact["schedule_contract"]["runtime_gain_interpolation"] == "not_run_until_all_source_nodes_complete"
    nodes = {node["point_id"]: node for node in artifact["nodes"]}
    assert nodes["3"]["status"] == "physical_surface_node_complete"
    assert all(nodes[node_id]["status"] == "physical_surface_node_boundary" for node_id in ("4", "5", "6", "7"))
    assert all(nodes[node_id]["status"] == "physical_surface_node_blocked" for node_id in ("8", "9", "10"))
    assert nodes["3"]["initialization"]["body_pitch_offset_from_condition3_deg"] == 0.0
    assert math.isclose(nodes["5"]["initialization"]["body_pitch_offset_from_condition3_deg"], 3.7, abs_tol=1.0e-12)
    assert artifact["source_deck_generation"]["attitude_initialization_policy"].startswith("preserve the checked-in condition-3 quaternion")
    assert "bounded_retrim_policy" in artifact["source_deck_generation"]
    ####


def test_b747_node_failure_has_machine_readable_remedy() -> None:
    """A blocked source node exposes a code and actionable integration hint."""

    failure = B747NodeIntegrationFailure(
        "source_trim_residual_above_tolerance",
        "source condition 4 trim did not converge",
        "Resolve the source operating-point trim convention before promotion.",
        {"worst_residual_name": "body_z_force_n"},
    )
    assert failure.code == "source_trim_residual_above_tolerance"
    assert failure.details["worst_residual_name"] == "body_z_force_n"
    assert "trim convention" in failure.hint
    ####


def test_b747_source_node_filter_keeps_altitude_grids_separate() -> None:
    """A node deck must not mix repeated source rows at different heights."""

    point = {"fc_id": "4", "mach": "0.65", "altitude_ft": "0.0"}
    assert _filter({"configuration": "nominal", "reference_fc_id": "4", "mach": "0.65", "altitude_ft": "0.0"}, point)
    assert not _filter({"configuration": "nominal", "reference_fc_id": "4", "mach": "0.65", "altitude_ft": "10000.0"}, point)
    ####


def test_b747_node_quaternion_matches_published_alpha_without_changing_fc3() -> None:
    """Non-FC3 initializers match source alpha in body velocity coordinates."""

    conditions = _read_conditions()
    for point_id in ("4", "5", "6", "7", "8", "9", "10"):
        point = conditions[point_id]
        body_velocity = _node_quaternion(point).conjugate().rotate(Vector3(0.0, 1.0, 0.0))
        alpha_deg = math.degrees(math.atan2(body_velocity.z, body_velocity.x))
        assert math.isclose(alpha_deg, float(point["alpha0_deg"]), abs_tol=1.0e-10)

    fc3_velocity = _node_quaternion(conditions["3"]).conjugate().rotate(Vector3(0.0, 1.0, 0.0))
    assert math.isclose(math.degrees(math.atan2(fc3_velocity.z, fc3_velocity.x)), 2.675223735133929, abs_tol=1.0e-12)
    ####
