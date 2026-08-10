from __future__ import annotations

from taoryx.horizontal_readiness import build_horizontal_readiness_report, preflight_horizontal_showcase


def test_horizontal_readiness_has_one_record_per_family_and_tier() -> None:
    report = build_horizontal_readiness_report()

    assert report.family_count == 9
    assert report.tier_count == 36
    assert report.status == "development"
    assert len({(item.family_id, item.tier) for item in report.records}) == 36


def test_horizontal_readiness_keeps_physical_claims_separate() -> None:
    report = build_horizontal_readiness_report()

    x15 = next(item for item in report.records if item.family_id == "x15" and item.tier == "rigid_body_6dof_direct_wrench")
    assert x15.adapter_status == "pass"
    assert "full_source_physical_effector_trim" in x15.blockers
    assert x15.data_status == "partial"
    assert x15.status == "development"

    tumbling = next(item for item in report.records if item.family_id == "tumbling_body" and item.tier == "rigid_body_6dof_direct_wrench")
    assert tumbling.status == "not_applicable"
    assert tumbling.profile_id is None


def test_horizontal_readiness_reports_a320_reduced_evidence_without_promoting_rigid_tiers() -> None:
    report = build_horizontal_readiness_report()

    a320 = [item for item in report.records if item.family_id == "a320_openap_3dof"]
    direct = next(item for item in a320 if item.tier == "rigid_body_6dof_direct_wrench")
    assert direct.declared_status == "planned"
    assert direct.status == "planned"

    point = next(item for item in a320 if item.tier == "point_mass_3dof")
    assert point.data_status == "ready"
    assert point.status == "probe_ready"
    assert point.data_source == "verification/daveml_a320_openap_integration.json"
    assert point.data_claim_boundary is not None
    assert "rigid-body moments" in point.data_claim_boundary

    pseudo = next(item for item in a320 if item.tier == "pseudo_6dof")
    assert pseudo.data_status == "ready"
    assert pseudo.status == "probe_ready"
    assert pseudo.data_claim_boundary is not None
    assert "physical surface allocation" in pseudo.data_claim_boundary


def test_showcase_preflight_allows_checked_x8_surface_path_without_promoting_it() -> None:
    report = build_horizontal_readiness_report()

    preflight = preflight_horizontal_showcase(
        "skywalker_x8",
        "rigid_body_6dof_surface_allocated",
        report=report,
        required_operations=("trim", "linearize", "effectiveness", "allocate"),
    )

    assert preflight.allowed is True
    assert preflight.readiness_status == "development"
    assert "end_to_end_surface_allocated_racetrack" in preflight.advisories


def test_showcase_preflight_allows_evidenced_a320_pseudo_path_without_promoting_it() -> None:
    report = build_horizontal_readiness_report()

    preflight = preflight_horizontal_showcase("a320_openap_3dof", "pseudo_6dof", report=report)

    assert preflight.allowed is True
    assert preflight.readiness_status == "probe_ready"
    assert "calibrated_response_model_and_operating_points" in preflight.advisories
