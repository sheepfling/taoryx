from __future__ import annotations

import math
from pathlib import Path

import pytest

from examples.showcases.california_to_hawaii.run_showcase import build_native_artifact, generate
from taoryx.language import GrammarProfile, parse_problem_file
from taoryx.language.semantic_validation import validate_problem
from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables
from taoryx.runtime.runner import run_files
from taoryx.vehicle import AeroQueryContext, PreparedAerodynamicCoefficients

pytestmark = [pytest.mark.artifact, pytest.mark.slow]


def test_california_to_hawaii_native_problem_contract_is_taoryx_6dof() -> None:
    problem = parse_problem_file(
        Path("examples/showcases/california_to_hawaii/mission.prb"),
        profile=GrammarProfile.TAORYX,
    )

    assert problem.diagnostics == []
    assert validate_problem(problem) == []
    mode = next(block for block in problem.problems[0].blocks if block.keyword == "mode")
    assert mode.mode == "rigid-body-6dof"
    assert problem.problems[0].trajectories[0].segments[-1].number == 5
    ####


def test_california_to_hawaii_aero_fixture_has_control_surface_axes() -> None:
    document = parse_table_file(Path("examples/showcases/california_to_hawaii/aero.tbl"))
    assert document.diagnostics == []
    tables = lower_tables(document)
    coefficients = PreparedAerodynamicCoefficients.from_runtime_tables(tables)
    force = coefficients.context_force_provider()
    result = force(AeroQueryContext.from_air_data(10.0, 0.0, 0.0, {"bank": 0.0, "fin_pitch": 0.0}))

    assert result.x < 0.0
    assert set(coefficients.force_tables) == {"cx", "cy", "cz"}
    assert set(coefficients.moment_tables) == {"cmx", "cmy", "cmz"}
    assert all(table.independent_variables == ("mach", "alpha", "bank", "fin_pitch") for table in coefficients.force_tables.values())
    ####


def test_california_to_hawaii_problem_executes_through_native_runner(tmp_path: Path) -> None:
    root = Path("examples/showcases/california_to_hawaii")
    report = run_files(
        root / "mission.prb",
        (root / "aero.tbl",),
        output_dir=tmp_path / "native-run",
        max_steps=3_000,
        profile=GrammarProfile.TAORYX,
    )

    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.cases == 1
    assert report.results[0].completed
    assert report.results[0].stop_reason == "stop_condition"
    final = report.results[0].states["1"][-1]
    assert final.time < 3_000.0
    assert final.named["pro_nav_acceleration_m_s2"] >= 0.0
    assert "alpha_command_deg" in final.named
    assert "bank_command_deg" in final.named
    assert final.named["aero_active"] == pytest.approx(1.0)
    assert "aero_dynamic_pressure_pa" in final.named
    assert "latitude_deg" in final.named
    assert "longitude_deg" in final.named
    assert "range_to_target_m" in final.named
    assert {"pro_nav_los_range_m", "pro_nav_closing_velocity_m_s", "pro_nav_los_azimuth_rate_deg_s", "pro_nav_los_elevation_rate_deg_s"} <= final.named.keys()
    assert {"pro_nav_command_ecfc_x_m_s2", "pro_nav_command_ecfc_y_m_s2", "pro_nav_command_ecfc_z_m_s2"} <= final.named.keys()
    assert {"pro_nav_achieved_aero_acceleration_m_s2", "pro_nav_acceleration_response_residual_m_s2"} <= final.named.keys()
    assert "pro_nav_active" in final.named
    assert "attitude_controller_saturated" in final.named
    assert report.metadata[0]["native_pipeline"]["integration_frame"] == "ecic"
    assert report.metadata[0]["native_pipeline"]["environment_frame"] == "ecfc"
    assert report.metadata[0]["native_pipeline"]["terminal_guidance_mode"] == "route"
    assert report.metadata[0]["native_pipeline"]["terminal_guidance_owner"] == "route"
    assert report.metadata[0]["thermal"]["heat-rate-coefficient"] == "0.002"
    assert report.metadata[0]["vehicle"]["dry-mass-kg"] == "8000.0"
    assert report.metadata[0]["actuator"]["maximum-body-rate-deg-s"] == "3600.0"
    assert report.metadata[0]["telemetry"]["sample-interval-s"] == "10.0"
    ####


def test_california_to_hawaii_native_target_hit_gate_is_independent(tmp_path: Path) -> None:
    """Check the native Earth-intersection state against independent target metrics."""

    root = Path("examples/showcases/california_to_hawaii")
    report = run_files(
        root / "mission.prb",
        (root / "aero.tbl",),
        output_dir=tmp_path / "native-acceptance",
        max_steps=3_000,
        profile=GrammarProfile.TAORYX,
    )
    final = report.results[0].states["1"][-1]
    start_latitude = math.radians(21.31)
    start_longitude = math.radians(-157.86)
    final_latitude = math.radians(final.named["latitude_deg"])
    final_longitude = math.radians(final.named["longitude_deg"])
    cosine = (
        math.sin(start_latitude) * math.sin(final_latitude)
        + math.cos(start_latitude) * math.cos(final_latitude) * math.cos(final_longitude - start_longitude)
    )
    great_circle_error_m = 6_378_137.0 * math.acos(max(-1.0, min(1.0, cosine)))

    assert report.exit_code == 0
    assert final.named["altitude_m"] <= 100.0
    assert great_circle_error_m <= 40_000.0
    ####


def test_california_to_hawaii_native_artifact_uses_runner_telemetry(tmp_path: Path) -> None:
    artifact = build_native_artifact(tmp_path / "native-artifact")

    assert artifact.vehicles
    vehicle = next(iter(artifact.vehicles.values()))
    source_names = {channel.source_name for channel in vehicle.channels.values()}
    assert {"latitude_deg", "longitude_deg", "altitude_m", "range_to_target_m"} <= source_names
    assert {"force_body_x_n", "moment_body_y_nm", "aero_dynamic_pressure_pa"} <= source_names
    assert vehicle.dynamics.value == "rigid_body_6dof"
    ####


def test_california_to_hawaii_artifact_rerun_writes_all_plot_views(tmp_path: Path) -> None:
    output = tmp_path / "california-to-hawaii"
    generate(output)

    expected = {
        "route_map.png",
        "altitude_range.png",
        "altitude_mach.png",
        "mass_propulsion.png",
        "thermal_exposure.png",
        "forces_torques.png",
        "guidance_response.png",
        "orientation.png",
        "attitude_rates.png",
    }
    assert {path.name for path in (output / "plots").glob("*.png")} == expected
    assert (output / "run.json").is_file()
    assert (output / "run.sqlite").is_file()
    assert (output / "summary.txt").is_file()
    ####
