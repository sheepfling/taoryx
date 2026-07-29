from __future__ import annotations

from taoryx.vehicle_controller_mission_preflight import validate_vehicle_controller_mission_preflight


def test_f16_preflight_checks_controller_paths_and_estimates_racetrack_time() -> None:
    report = validate_vehicle_controller_mission_preflight("reference_f16_s119")

    assert report.status == "development"
    assert report.controller.status == "development"
    assert len(report.controller.profiles_checked) == 2
    assert report.mission.status == "development"
    assert report.mission.metrics["estimated_duration_s_min"] == 1408.1865661383833
    assert any(item.code == "controller-effectors-unresolved" for item in report.controller.findings)
    assert any(item.code == "mission-direct-wrench-realization" for item in report.mission.findings)
    ####


def test_hl20_preflight_fails_closed_without_a_mission_binding() -> None:
    report = validate_vehicle_controller_mission_preflight("reference_hl20_mod_k")

    assert report.status == "blocked"
    assert report.controller.status == "not_applicable"
    assert report.mission.status == "blocked"
    assert any(item.code == "mission-binding-missing" for item in report.blockers)
    ####
