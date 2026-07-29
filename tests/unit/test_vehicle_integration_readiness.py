"""Tests for provider-neutral source-family integration readiness."""

from __future__ import annotations

from taoryx.vehicle_integration_readiness import (
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)


def test_f16_readiness_reports_all_declared_profiles_without_lowering() -> None:
    """F-16 metadata is ready for probes but development status is not promotion."""

    report = validate_vehicle_integration_readiness("reference_f16_s119")
    assert report.status == "ready_for_runtime_probes"
    assert not report.errors
    assert len(report.profiles) == 4
    assert all(profile.metadata_ready for profile in report.profiles)
    assert not any(profile.automatically_lowerable for profile in report.profiles)
    ####


def test_hl20_missing_direct_wrench_profile_is_explicit() -> None:
    """Missing physical tiers become a blocker instead of an implicit fallback."""

    report = validate_vehicle_integration_readiness("reference_hl20_mod_k")
    direct = next(profile for profile in report.profiles if profile.profile_id.endswith("rigid_body_6dof_direct_wrench"))
    assert report.status == "blocked"
    assert direct.declared is False
    assert any(finding.code == "profile-not-declared" for finding in direct.findings)
    ####


def test_all_supported_reference_families_have_reports() -> None:
    """The readiness layer enumerates the provider-neutral family registry."""

    reports = validate_all_vehicle_integration_readiness()
    assert {report.family_id for report in reports} == {"reference_f16_s119", "reference_hl20_mod_k"}
    ####
