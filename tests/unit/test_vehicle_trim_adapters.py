from __future__ import annotations

from taoryx.vehicle_trim_adapters import solve_vehicle_trim_evidence


def test_f16_adapter_solves_all_declared_catalog_points() -> None:
    report = solve_vehicle_trim_evidence("reference_f16_s119")

    assert report.status == "verified"
    assert len(report.points) == 7
    assert all(point.success for point in report.points)
    assert max(point.max_residual for point in report.points) < 1.0e-6
    ####


def test_hl20_adapter_solves_source_bounded_pitch_point() -> None:
    report = solve_vehicle_trim_evidence("reference_hl20_mod_k")

    assert report.status == "verified"
    assert len(report.points) == 1
    assert report.points[0].point_id == "hl20-mach1-pitch-trim"
    assert report.points[0].max_residual < 1.0e-10
    ####
