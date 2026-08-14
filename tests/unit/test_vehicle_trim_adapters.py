from __future__ import annotations

from taoryx.plugins import discover_plugins
from taoryx.vehicle_trim_adapters import solve_vehicle_trim_evidence


def test_f16_adapter_solves_all_declared_catalog_points() -> None:
    plugins = discover_plugins(
        include_external=False,
        selected=("taoryx.daveml", "taoryx.f16"),
    )
    report = solve_vehicle_trim_evidence("reference_f16_s119", plugins=plugins)

    assert report.status == "verified"
    assert len(report.points) == 7
    assert all(point.success for point in report.points)
    assert max(point.max_residual for point in report.points) < 1.0e-6
    ####


def test_hl20_adapter_solves_source_bounded_pitch_point() -> None:
    plugins = discover_plugins(
        include_external=False,
        selected=("taoryx.daveml", "taoryx.hl20"),
    )
    report = solve_vehicle_trim_evidence("reference_hl20_mod_k", plugins=plugins)

    assert report.status == "verified"
    assert len(report.points) == 1
    assert report.points[0].point_id == "hl20-mach1-pitch-trim"
    assert report.points[0].max_residual < 1.0e-10
    ####


def test_trim_evidence_does_not_escape_the_selected_plugin_scope() -> None:
    """A focused host must not recover an unselected family through core."""

    plugins = discover_plugins(
        include_external=False,
        selected=("taoryx.daveml", "taoryx.f16"),
    )
    report = solve_vehicle_trim_evidence("reference_hl20_mod_k", plugins=plugins)

    assert report.status == "blocked"
    assert report.findings == ("no selected plug-in registers trim evidence for this source family",)
    ####
