"""Tests for provider-neutral source-family integration readiness."""

from __future__ import annotations

import json

import pytest

from taoryx.runtime.cli import main
from taoryx.vehicle_integration_readiness import (
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)


def test_f16_readiness_resolves_canonical_profiles_by_realization() -> None:
    """F-16 metadata resolves one explicit profile for each canonical tier."""

    report = validate_vehicle_integration_readiness("reference_f16_s119")
    assert report.status == "ready_for_runtime_probes"
    assert not report.errors
    assert len(report.profiles) == 4
    assert all(profile.metadata_ready for profile in report.profiles)
    direct = next(profile for profile in report.profiles if profile.tier == "rigid_body_6dof_direct_wrench")
    assert direct.profile_id == "f16.rigid_body_6dof.v1"
    assert direct.automatically_lowerable
    ####


def test_hl20_direct_wrench_profile_is_resolved_by_declared_realization() -> None:
    """A base profile can fill a canonical tier when its realization is explicit."""

    report = validate_vehicle_integration_readiness("reference_hl20_mod_k")
    direct = next(profile for profile in report.profiles if profile.tier == "rigid_body_6dof_direct_wrench")
    assert report.status == "ready_for_runtime_probes"
    assert direct.declared is True
    assert direct.profile_id == "hl20.rigid_body_6dof.v1"
    assert not any(finding.code == "profile-not-declared" for finding in direct.findings)
    ####


def test_all_supported_reference_families_have_reports() -> None:
    """The readiness layer enumerates the provider-neutral family registry."""

    reports = validate_all_vehicle_integration_readiness()
    assert {report.family_id for report in reports} == {"reference_f16_s119", "reference_hl20_mod_k"}
    ####


def test_mission_composition_cli_exposes_source_readiness_without_promoting_a_family(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """A composition author can discover source onboarding gaps from the public CLI."""

    exit_code = main(["vehicle", "integration", "readiness", "reference_f16_s119"])

    assert exit_code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["schema"] == "taoryx.vehicle-integration-command/v1alpha1"
    assert payload["command"] == "readiness"
    assert "does not execute a source plant" in payload["claim_boundary"]
    assert payload["reports"][0]["family_id"] == "reference_f16_s119"
    assert payload["reports"][0]["status"] == "ready_for_runtime_probes"
    ####
