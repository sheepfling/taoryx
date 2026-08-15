from __future__ import annotations

import pytest

from taoryx.horizontal_fidelity import load_horizontal_registry
from tools.validate_family_adapter_registry import build_registry, build_report


@pytest.mark.slow
@pytest.mark.integration
def test_registry_witness_report_separates_executable_and_planned_families() -> None:
    registry = build_registry()
    report = build_report()

    assert report["status"] == "development"
    assert report["executable_witness_count"] == sum(item.status == "available" for item in registry.registrations)
    assert report["planned_family_count"] == sum(item.status == "planned" for item in registry.registrations)
    assert report["alignment"]["status"] == "pass"
    assert report["horizontal_lowering"]["status"] == "pass"
    lowering_by_family = {item["family_id"]: item for item in report["horizontal_lowering"]["families"]}
    assert lowering_by_family["skywalker_x8"]["lowering"]["selected"] == "rigid_body_6dof_direct_wrench"
    assert lowering_by_family["hl20_mod_k"]["lowering"]["selected"] == "rigid_body_6dof_direct_wrench"
    assert report["declared_tier_check_count"] == sum(
        len(item.supported_tiers) for item in registry.registrations if item.status == "available"
    )
    assert report["tier_matrix"]["status"] == "pass"
    promotion = report["promotion_matrix"]
    assert len(promotion["entries"]) == sum(len(family.tiers) for family in load_horizontal_registry().families)
    assert len(promotion["entries"]) == sum(promotion["declared_status_counts"].values())
    assert len(promotion["entries"]) == sum(promotion["validation_status_counts"].values())
    assert promotion["qualified_failures"] == []
    assert promotion["declared_status_counts"]["planned"] == sum(
        binding.promotion_status == "planned"
        for family in load_horizontal_registry().families
        for binding in family.tiers.values()
    )

    checks = report["registry"]["checks"]
    by_family = {item["family_id"]: item for item in checks}
    assert by_family["skywalker_x8"]["status"] == "pass"
    assert by_family["b747"]["status"] == "pass"
    assert by_family["hummingbird"]["status"] == "pass"
    assert by_family["f16_s119"]["status"] == "pass"
    assert by_family["a320_openap_3dof"]["status"] == "pass"
    assert by_family["tumbling_body"]["status"] == "pass"
    assert by_family["reference_nesc_two_stage_rocket"]["status"] == "pass"
    nesc_operations = {item["operation"]: item["status"] for item in by_family["reference_nesc_two_stage_rocket"]["probe"]["operations"]}
    assert nesc_operations["replay"] == "pass"
    assert nesc_operations["state_derivative"] == "not_applicable"
    assert nesc_operations["allocate"] == "not_applicable"
    x15_operations = {item["operation"]: item["status"] for item in by_family["x15"]["probe"]["operations"]}
    assert x15_operations["state_derivative"] == "pass"
    assert x15_operations["trim"] == "pass"
    assert x15_operations["linearize"] == "pass"
    assert x15_operations["effectiveness"] == "not_applicable"
    assert x15_operations["allocate"] == "not_applicable"

    for family_id in ("skywalker_x8", "b747", "hummingbird", "f16_s119"):
        operations = {item["operation"]: item["status"] for item in by_family[family_id]["probe"]["operations"]}
        assert operations["state_derivative"] == "pass"
        assert operations["trim"] == "pass"
        assert operations["linearize"] == "pass"
        assert operations["effectiveness"] == "pass"
        assert operations["allocate"] == "pass"

    a320_operations = {item["operation"]: item["status"] for item in by_family["a320_openap_3dof"]["probe"]["operations"]}
    assert a320_operations["state_derivative"] == "pass"
    assert a320_operations["trim"] == "pass"
    assert a320_operations["linearize"] == "pass"
    assert a320_operations["effectiveness"] == "not_applicable"
    assert a320_operations["allocate"] == "not_applicable"

    matrix_by_key = {(item["family_id"], item["tier"]): item for item in report["tier_matrix"]["checks"]}
    x15_surface_operations = {
        item["operation"]: item["status"]
        for item in matrix_by_key[("x15", "rigid_body_6dof_surface_allocated")]["probe"]["operations"]
    }
    assert x15_surface_operations["state_derivative"] == "pass"
    assert x15_surface_operations["trim"] == "pass"
    assert x15_surface_operations["linearize"] == "pass"
    assert x15_surface_operations["effectiveness"] == "pass"
    assert x15_surface_operations["allocate"] == "pass"
    for family_id in ("hummingbird", "f16_s119"):
        for tier in ("point_mass_3dof", "pseudo_6dof"):
            operations = {
                item["operation"]: item["status"]
                for item in matrix_by_key[(family_id, tier)]["probe"]["operations"]
            }
            assert operations["state_derivative"] == "pass"
            assert operations["trim"] == "pass"
            assert operations["linearize"] == "pass"
            assert operations["effectiveness"] == "not_applicable"
            assert operations["allocate"] == "not_applicable"

    hl20_operations = {item["operation"]: item["status"] for item in by_family["hl20_mod_k"]["probe"]["operations"]}
    assert hl20_operations["state_derivative"] == "pass"
    assert hl20_operations["trim"] == "pass"
    assert hl20_operations["trim_fragment"] == "pass"
    assert hl20_operations["linearize"] == "pass"
    assert hl20_operations["effectiveness"] == "pass"
    assert hl20_operations["allocate"] == "pass"

    promotion_by_key = {(item["family_id"], item["tier"]): item for item in promotion["entries"]}
    hl20_direct = promotion_by_key[("hl20_mod_k", "rigid_body_6dof_direct_wrench")]
    assert hl20_direct["validation_status"] == "pass"
    x8_direct = promotion_by_key[("skywalker_x8", "rigid_body_6dof_direct_wrench")]
    assert x8_direct["declared_status"] == "development"
    assert x8_direct["validation_status"] == "pass"
    assert "envelope_robustness_and_flight_qualification" in x8_direct["blockers"]
    nesc_direct = promotion_by_key[("reference_nesc_two_stage_rocket", "rigid_body_6dof_direct_wrench")]
    assert nesc_direct["declared_status"] == "planned"
    assert nesc_direct["validation_status"] == "planned"
    x15_surface = promotion_by_key[("x15", "rigid_body_6dof_surface_allocated")]
    assert x15_surface["declared_status"] == "development"
    assert x15_surface["validation_status"] == "pass"
    assert "source_trim_acceptance" in x15_surface["blockers"]
    tumbling_surface = promotion_by_key[("tumbling_body", "rigid_body_6dof_surface_allocated")]
    assert tumbling_surface["validation_status"] == "not_applicable"
