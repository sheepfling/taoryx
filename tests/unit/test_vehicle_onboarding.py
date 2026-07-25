from __future__ import annotations

from taoryx.vehicle_onboarding import validate_all_vehicle_onboarding, validate_vehicle_onboarding


def test_all_registered_vehicles_have_a_diagnosable_onboarding_report() -> None:
    """The standard catalog is complete even when a vehicle remains provisional."""

    reports = validate_all_vehicle_onboarding()
    assert {report.vehicle_id for report in reports} == {"b747", "skywalker_x8", "hummingbird", "x15"}
    assert all(not report.errors for report in reports)
    assert validate_vehicle_onboarding("x15").status == "ready"
    ####


def test_unknown_vehicle_error_identifies_the_registration_path_and_repair() -> None:
    """A typo produces a useful catalog error rather than a bare KeyError."""

    report = validate_vehicle_onboarding("new_vehicle")
    assert report.status == "blocked"
    assert len(report.errors) == 1
    finding = report.errors[0]
    assert finding.code == "unknown-vehicle-id"
    assert "vehicle_models.yaml:vehicles.new_vehicle" in finding.path
    assert "Add it under vehicles" in finding.hint
    ####


def test_onboarding_reports_are_json_ready() -> None:
    """A clean onboarding report remains machine-readable."""

    report = validate_vehicle_onboarding("x15")
    payload = report.as_dict()
    assert payload["status"] == "ready"
    assert payload["findings"] == []
    ####
