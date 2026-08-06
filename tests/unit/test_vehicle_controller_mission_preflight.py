from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.vehicle_controller_mission_preflight import (
    PreflightFinding,
    _authority_preflight_for_controller,
    validate_vehicle_controller_mission_preflight,
)


def test_f16_preflight_checks_controller_paths_and_estimates_racetrack_time() -> None:
    report = validate_vehicle_controller_mission_preflight("reference_f16_s119")

    assert report.status == "development"
    assert report.controller.status == "development"
    assert len(report.controller.profiles_checked) == 2
    assert report.mission.status == "development"
    # The route resolver places climb/descent inside the straight legs.  The
    # preflight must therefore use the shared horizon rather than add vertical
    # times a second time.
    assert report.mission.metrics["estimated_duration_s_min"] == pytest.approx(1308.1865661383836)
    authority = report.controller.metrics["authority_preflights"]
    assert set(authority) == {
        "f16.local_lqr_trim_hold.v1",
        "f16.local_physical_wrench_lqr.v1",
    }
    assert all(item["status"] == "passed" for item in authority.values())
    assert all(item["metrics"]["controllability_rank"] == 6.0 for item in authority.values())
    assert any(item.code == "controller-effectors-unresolved" for item in report.controller.findings)
    assert any(item.code == "mission-direct-wrench-realization" for item in report.mission.findings)
    ####


def test_hl20_preflight_fails_closed_without_a_mission_binding() -> None:
    report = validate_vehicle_controller_mission_preflight("reference_hl20_mod_k")

    assert report.status == "development"
    assert report.controller.status == "not_applicable"
    assert report.mission.status == "development"
    assert any(item.code == "mission-runtime-planned" for item in report.mission.findings)
    assert not report.blockers
    ####


def test_lqr_profile_without_declared_authority_requirement_fails_closed(tmp_path: Path) -> None:
    """Onboarding may not begin an LQR sweep without an authority contract."""

    findings: list[PreflightFinding] = []
    result = _authority_preflight_for_controller(
        tmp_path / "missing-authority.yaml",
        {"method": "continuous_lqr"},
        None,
        findings,
    )

    assert result is None
    assert len(findings) == 1
    assert findings[0].severity == "error"
    assert findings[0].code == "controller-authority-requirement-missing"
