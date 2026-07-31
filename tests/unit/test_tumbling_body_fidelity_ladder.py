"""Tests for the common tumbling-body fidelity wrapper."""

from __future__ import annotations

import json
from pathlib import Path

from tools.validate_tumbling_body_fidelity_ladder import write_tumbling_body_fidelity_ladder


def test_tumbling_body_fidelity_wrapper_preserves_passive_boundary(tmp_path: Path) -> None:
    source = Path("verification/alpha3_tumbling_body/qualification.json")
    manifest = write_tumbling_body_fidelity_ladder(source, tmp_path)

    assert manifest["status"] == "nominal_case_pass"
    assert len(manifest["records"]) == 2
    pseudo = json.loads((tmp_path / "pseudo_6dof_evidence.json").read_text(encoding="utf-8"))
    comparison = json.loads((tmp_path / "comparison.json").read_text(encoding="utf-8"))
    assert pseudo["control_path"]["controller"] == "not_applicable_passive_body"
    assert pseudo["control_path"]["realization"] == "rigid_body_6dof_reuse"
    assert all(item["truth_result"] == "PASS" for item in pseudo["mission"]["required_objectives"])
    assert pseudo["qualification"]["controller_applicability"] == "not_applicable_passive_body"
    assert pseudo["qualification"]["family_status"] == "qualification_pending_parent_capability_and_source_exact_aerodynamics"
    assert comparison["loss_definition"]["reference"] == "point_mass_3dof_orientation_averaged_area"
    assert comparison["loss_definition"]["comparison"] == "pseudo_6dof_native_rigid_body_reuse"
    assert comparison["loss_definition"]["not_a_score"] is True
    assert {item["shape"] for item in comparison["records"]} == {
        "sphere",
        "cylinder",
        "cone",
        "triaxial_ellipsoid",
    }
    assert all("terminal_delta" in item and "area_policy_loss" in item for item in comparison["records"])
    ####
