from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import (
    PhaseWindow,
    independent_force_closure,
    independent_moment_closure,
    integral_mass_balance_error,
    require_bounded,
    require_monotonic,
    require_net_change,
)
from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, verify_golden_plant

ROOT = Path(__file__).resolve().parents[2]
B747_PROBLEM = ROOT / "examples/mission_families/slower_b747/SV01_rectangle_course_3dof.prb"
B747_TRIM_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_long_trim_hold_6dof.prb"
B747_TRIM_120_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_long_trim_hold_120_6dof.prb"
B747_DESCENT_RECOVERY_120_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_descent_recovery_120_6dof.prb"
B747_APPROACH_LONG_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_approach_go_around_long_6dof.prb"
B747_CLIMB_CRUISE_3DOF = ROOT / "examples/mission_families/slower_b747/SV01_climb_cruise_3dof.prb"
B747_REDUCTION_3DOF = ROOT / "examples/mission_families/slower_b747/SV01_powered_trim_reduction_3dof.prb"
B747_PARITY_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_powered_trim_parity_6dof.prb"
B747_PITCH_DIAGNOSTIC_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_open_loop_pitch_diagnostic_6dof.prb"
B747_DESCENT_DIAGNOSTIC_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_open_loop_descent_diagnostic_6dof.prb"
B747_FLIGHT_PATH_DESCENT_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_flight_path_hold_descent_6dof.prb"
B747_INTEGRATED_ROUTE_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_integrated_route_descent_6dof.prb"
B747_FUEL_MASS_DIAGNOSTIC_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_reduced_fuel_mass_coupling_6dof.prb"
B747_TABLES = (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_static_6axis.tbl",
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_jt9d_thrust.tbl",
)
B747_TRIM_TABLES = (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_elevator_6axis.tbl",
)
B747_REDUCTION_TABLES = B747_TRIM_TABLES + (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_jt9d_thrust.tbl",
)
B747_FUEL_TABLES = B747_TRIM_TABLES + (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_notional_fuel_flow.tbl",
)

pytestmark = [pytest.mark.slow, pytest.mark.b747]


def _run_b747(output_dir: Path) -> RunReport:
    """Run the executable fixed-wing long candidate at the file boundary."""

    return run_files(
        B747_PROBLEM,
        B747_TABLES,
        output_dir=output_dir,
        max_steps=20_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_trim_6dof(output_dir: Path) -> RunReport:
    """Run the source-anchored six-degree-of-freedom trim hold."""

    return run_files(
        B747_TRIM_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=4_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_trim_120_6dof(output_dir: Path) -> RunReport:
    """Run the metadata-generated 120-second source-anchor trim hold."""

    return run_files(
        B747_TRIM_120_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=7_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_descent_recovery_120_6dof(output_dir: Path) -> RunReport:
    """Run the generated 120-second descent and level-recovery maneuver."""

    return run_files(
        B747_DESCENT_RECOVERY_120_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=8_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_approach_long_6dof(output_dir: Path) -> RunReport:
    """Run the long approach/go-around phase sequence."""

    return run_files(
        B747_APPROACH_LONG_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=4_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_climb_cruise_3dof(output_dir: Path) -> RunReport:
    """Run the long point-mass climb/cruise reduction."""

    return run_files(
        B747_CLIMB_CRUISE_3DOF,
        B747_TABLES,
        output_dir=output_dir,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_reduction_3dof(output_dir: Path) -> RunReport:
    """Run the source-deck point-mass reduction at the rigid-body anchor."""

    return run_files(
        B747_REDUCTION_3DOF,
        B747_REDUCTION_TABLES,
        output_dir=output_dir,
        max_steps=100,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_pitch_diagnostic(elevator_deg: float, output_dir: Path) -> RunReport:
    """Run one small open-loop elevator perturbation from the source anchor."""

    problem = output_dir / f"b747-pitch-{elevator_deg:+.3f}.prb"
    problem.parent.mkdir(parents=True, exist_ok=True)
    problem.write_text(
        B747_PITCH_DIAGNOSTIC_6DOF.read_text(encoding="utf-8").replace(
            "default=0.3861602391", f"default={elevator_deg:.12g}"
        ),
        encoding="utf-8",
    )
    return run_files(
        problem,
        B747_TRIM_TABLES,
        output_dir=output_dir / f"run-{elevator_deg:+.3f}",
        max_steps=200,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_descent_diagnostic(output_dir: Path) -> RunReport:
    """Run the source-aligned short open-loop descending plant case."""

    return run_files(
        B747_DESCENT_DIAGNOSTIC_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_flight_path_descent(output_dir: Path) -> RunReport:
    """Run the source-scoped 60-second controlled descending corridor."""

    return run_files(
        B747_FLIGHT_PATH_DESCENT_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=8_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_integrated_route(output_dir: Path) -> RunReport:
    """Run the native route plus altitude-transition controller case."""

    return run_files(
        B747_INTEGRATED_ROUTE_6DOF,
        B747_TRIM_TABLES,
        output_dir=output_dir,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_b747_fuel_mass_diagnostic(output_dir: Path) -> RunReport:
    """Run the explicit reduced-order runtime mass-flow contract."""

    return run_files(
        B747_FUEL_MASS_DIAGNOSTIC_6DOF,
        B747_FUEL_TABLES,
        output_dir=output_dir,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _trim_case() -> GoldenPlantCase:
    """Describe the source-anchor case for the reusable plant firewall."""

    return GoldenPlantCase(
        vehicle="B747-long-trim",
        problem=B747_TRIM_6DOF,
        tables=B747_TRIM_TABLES,
        max_steps=4_000,
        alpha_beta_reference=(2.675223735133929, 0.0),
    )
####


def _approach_case() -> GoldenPlantCase:
    """Describe the long approach/go-around plant for convergence evidence."""

    return GoldenPlantCase(
        vehicle="B747-approach-go-around",
        problem=B747_APPROACH_LONG_6DOF,
        tables=B747_TRIM_TABLES,
        max_steps=4_000,
        alpha_beta_reference=(2.675223735133929, 0.0),
    )
####


def _history(report: RunReport) -> tuple[dict[str, float], ...]:
    assert report.results and report.results[0].completed
    states = report.results[0].states["1"]
    return tuple({"time_s": state.time, **dict(state.named)} for state in states)
####


def _pitch_diagnostic_history(report: RunReport) -> tuple[dict[str, float], ...]:
    """Add independently derived vertical-rate and flight-path telemetry."""

    history = list(_history(report))
    enriched: list[dict[str, float]] = []
    for index, sample in enumerate(history):
        if index == 0:
            altitude_rate = (history[1]["altitude_m"] - sample["altitude_m"]) / (history[1]["time_s"] - sample["time_s"])
        else:
            altitude_rate = (sample["altitude_m"] - history[index - 1]["altitude_m"]) / (sample["time_s"] - history[index - 1]["time_s"])
        horizontal_speed = math.sqrt(max(sample["speed_m_s"] ** 2 - altitude_rate**2, 0.0))
        enriched.append(
            {
                **sample,
                "raw_position_x_m": sample["x"],
                "raw_position_y_m": sample["y"],
                "raw_position_z_m": sample["z"],
                "vertical_velocity_m_s": altitude_rate,
                "flight_path_angle_deg": math.degrees(math.atan2(altitude_rate, horizontal_speed)),
            }
        )
    return tuple(enriched)
####


def test_b747_open_loop_pitch_control_effectiveness(tmp_path: Path) -> None:
    """Opposite elevator perturbations produce opposite initial pitch responses."""

    nose_down = _run_b747_pitch_diagnostic(-1.0, tmp_path / "nose-down")
    nose_up = _run_b747_pitch_diagnostic(1.8, tmp_path / "nose-up")
    for report in (nose_down, nose_up):
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]

    down_history = _pitch_diagnostic_history(nose_down)
    up_history = _pitch_diagnostic_history(nose_up)
    down_initial, up_initial = down_history[0], up_history[0]
    down_response, up_response = down_history[10], up_history[10]
    assert down_initial["aero_query_elevator-deg"] == pytest.approx(-1.0)
    assert up_initial["aero_query_elevator-deg"] == pytest.approx(1.8)
    assert down_initial["aero_moment_body_y_nm"] > 0.0
    assert up_initial["aero_moment_body_y_nm"] < 0.0
    assert down_response["local_pitch_deg"] > down_initial["local_pitch_deg"]
    assert up_response["local_pitch_deg"] < up_initial["local_pitch_deg"]
    assert down_response["vertical_velocity_m_s"] < 0.0
    assert up_response["vertical_velocity_m_s"] > 0.0
    for history in (down_history, up_history):
        require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
        require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
        require_bounded(history, "elevator-deg", minimum=-10.0, maximum=10.0)
        require_bounded(history, "aero_table_margin_moment.cmy.elevator", minimum=0.0)
####


def test_b747_open_loop_descent_is_kinematically_consistent(tmp_path: Path) -> None:
    """The aligned initial state produces a bounded short descending response."""

    report = _run_b747_descent_diagnostic(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _pitch_diagnostic_history(report)
    assert history[-1]["time_s"] == pytest.approx(10.0, abs=1.0e-10)
    require_monotonic(history, "altitude_m", "decreasing", tolerance=1.0e-9)
    require_net_change(history, "altitude_m", "decreasing", minimum=50.0)
    require_bounded(history, "speed_m_s", minimum=150.0, maximum=161.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "flight_path_angle_deg", minimum=-5.0, maximum=0.0)
    require_bounded(history, "elevator-deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
####


@pytest.mark.artifact
def test_b747_open_loop_descent_writes_standard_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the short descent plant response and its derived flight path."""

    report = _run_b747_descent_diagnostic(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-open-loop-descent-diagnostic"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "descent-history.json").write_text(
        json.dumps(
            {
                "contract": "short source-aligned open-loop descending plant response",
                "not_validated_as": "sustained descent equilibrium",
                "history": _pitch_diagnostic_history(report),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    assert (destination / "descent-history.json").stat().st_size > 1_000
####


def test_b747_flight_path_hold_descent_remains_in_source_corridor(tmp_path: Path) -> None:
    """A native flight-path command produces a bounded, descending 60-second case."""

    report = _run_b747_flight_path_descent(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _pitch_diagnostic_history(report)
    assert history[-1]["time_s"] == pytest.approx(60.0, abs=1.0e-10)
    require_monotonic(history, "altitude_m", "decreasing", tolerance=1.0e-9)
    require_net_change(history, "altitude_m", "decreasing", minimum=700.0)
    require_bounded(history, "altitude_m", minimum=0.0)
    require_bounded(history, "speed_m_s", minimum=150.0, maximum=200.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "aero_mach", minimum=0.0, maximum=0.9)
    require_bounded(history, "flight_path_angle_deg", minimum=-6.0, maximum=-3.0)
    require_bounded(history, "aero_table_margin_moment.cmy.elevator", minimum=0.0)
    require_bounded(history, "attitude_controller_saturated", minimum=0.0, maximum=0.0)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
####


def test_b747_reduced_fuel_mass_coupling_is_conservative(tmp_path: Path) -> None:
    """The explicit research surrogate mdot decreases mass exactly once."""

    report = _run_b747_fuel_mass_diagnostic(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(10.0, abs=1.0e-10)
    require_monotonic(history, "mass_kg", "decreasing", tolerance=1.0e-10)
    require_monotonic(history, "propellant_mass_kg", "decreasing", tolerance=1.0e-10)
    assert history[0]["mass_kg"] == pytest.approx(288876.9)
    assert history[-1]["mass_kg"] == pytest.approx(288856.9, abs=1.0e-6)
    assert history[-1]["propellant_mass_kg"] == pytest.approx(100.0, abs=1.0e-6)
    assert max(sample["propellant_mass_rate_kg_s"] for sample in history) == pytest.approx(2.0)
    assert integral_mass_balance_error(history, mass_rate_channel="propellant_mass_rate_kg_s") < 1.0e-8
    require_bounded(history, "propellant_mass_kg", minimum=0.0)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
####


def test_b747_integrated_route_and_altitude_transition_are_bounded(tmp_path: Path) -> None:
    """Route and altitude guidance compose without leaving the local deck."""

    report = _run_b747_integrated_route(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time"] == pytest.approx(20.0, abs=1.0e-10)
    assert history[-1]["range_to_target_m"] < 400.0
    require_bounded(history, "altitude_m", minimum=95.0, maximum=270.0)
    require_bounded(history, "speed_m_s", minimum=140.0, maximum=170.0)
    require_bounded(history, "aero_alpha_deg", minimum=-4.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-4.0, maximum=4.0)
    require_bounded(history, "aero_mach", minimum=0.0, maximum=0.9)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
####


@pytest.mark.artifact
def test_b747_integrated_route_writes_standard_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the composed route and altitude-transition evidence."""

    report = _run_b747_integrated_route(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-integrated-route-descent"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.range_to_target_m",
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.aero_alpha_deg",
                "taos.aero_sideslip_deg",
                "taos.mach",
                "taos.flight_path_angle_deg",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_controlled_descent_writes_standard_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the long controlled-descent corridor evidence."""

    report = _run_b747_flight_path_descent(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-flight-path-hold-descent"
    destination.mkdir(parents=True, exist_ok=True)
    history = _pitch_diagnostic_history(report)
    (destination / "descent-history.json").write_text(
        json.dumps(
            {
                "contract": "60-second source-scoped controlled descending corridor",
                "claim_boundary": "research surrogate; not a global B747 validation",
                "target_flight_path_angle_deg": -3.0,
                "accepted_flight_path_angle_deg": [-6.0, -3.0],
                "history": history,
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.aero_alpha_deg",
                "taos.aero_sideslip_deg",
                "taos.flight_path_angle_deg",
                "taos.mach",
                "taos.thrust_n",
                "taos.mass_kg",
            ),
        )
    assert (destination / "descent-history.json").stat().st_size > 10_000
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_fuel_mass_coupling_writes_standard_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the reduced-order mass-flow contract and its explicit caveat."""

    report = _run_b747_fuel_mass_diagnostic(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-reduced-fuel-mass-coupling"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "mass-history.json").write_text(
        json.dumps(
            {
                "contract": "runtime mass decreases by the explicit mdot assignment",
                "claim_boundary": "reduced-order diagnostic; JT9D source table has no fuel-flow law",
                "history": _history(report),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=("taos.mass_kg", "taos.propellant_mass_kg", "taos.propellant_mass_rate_kg_s", "taos.thrust_n"),
        )
    assert (destination / "mass-history.json").stat().st_size > 1_000
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_open_loop_pitch_diagnostic_writes_standard_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the raw and derived vertical/pitch diagnostic channels."""

    reports = {
        "elevator_minus_1deg": _run_b747_pitch_diagnostic(-1.0, tmp_path / "artifact-minus"),
        "elevator_plus_1p8deg": _run_b747_pitch_diagnostic(1.8, tmp_path / "artifact-plus"),
    }
    destination = artifact_dir / "vehicle-family-validation" / "b747-open-loop-pitch-diagnostic"
    destination.mkdir(parents=True, exist_ok=True)
    payload = {
        "contract": "opposite elevator perturbations produce opposite initial pitching responses",
        "channels": [
            "raw_position_x_m",
            "raw_position_y_m",
            "raw_position_z_m",
            "altitude_m",
            "vertical_velocity_m_s",
            "flight_path_angle_deg",
            "local_pitch_deg",
            "aero_alpha_deg",
            "aero_moment_body_y_nm",
            "elevator-deg",
            "aero_query_elevator-deg",
            "thrust_n",
            "drag_force_n",
            "elevator_controller_saturated",
        ],
        "cases": {},
    }
    for case, report in reports.items():
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        payload["cases"][case] = _pitch_diagnostic_history(report)
    (destination / "diagnostic-history.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    assert (destination / "diagnostic-history.json").stat().st_size > 1_000
####


def test_b747_long_candidate_executes_four_geographic_legs(tmp_path: Path) -> None:
    """The fixed-wing candidate completes a long rectangle with expected leg signs."""

    report = _run_b747(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert len(history) > 1_000
    assert history[-1]["time"] == pytest.approx(320.0, abs=1.0e-10)

    phases = (
        PhaseWindow("leg-1", 0.0, 80.0),
        PhaseWindow("leg-2", 80.0, 160.0),
        PhaseWindow("leg-3", 160.0, 240.0),
        PhaseWindow("leg-4", 240.0, 320.0),
    )
    leg_samples = [phase.select(history) for phase in phases]
    require_net_change(leg_samples[0], "long", "increasing", minimum=0.01)
    require_net_change(leg_samples[1], "lat", "increasing", minimum=0.01)
    require_net_change(leg_samples[2], "long", "decreasing", minimum=0.005)
    require_net_change(leg_samples[3], "lat", "decreasing", minimum=0.005)

    require_bounded(history, "alt", minimum=0.0, maximum=500.0)
    require_bounded(history, "vel", minimum=0.0, maximum=600.0)
    require_bounded(history, "mach", minimum=0.0, maximum=0.6)
    assert history[-1]["long"] < leg_samples[1][-1]["long"]
####


def test_b747_long_climb_cruise_has_expected_energy_phase(tmp_path: Path) -> None:
    """The long point-mass fixed-wing case climbs while retaining airspeed."""

    report = _run_b747_climb_cruise_3dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time"] == pytest.approx(60.0, abs=1.0e-10)
    require_net_change(history, "alt", "increasing", minimum=20.0)
    require_net_change(history, "long", "increasing", minimum=0.005)
    require_bounded(history, "alt", minimum=300.0, maximum=450.0)
    require_bounded(history, "vel", minimum=480.0, maximum=520.0)
    require_bounded(history, "mach", minimum=0.0, maximum=0.6)
####


def test_b747_long_6dof_trim_hold_is_bounded(tmp_path: Path) -> None:
    """The source-anchored 6DOF transport plant stays in its declared envelope."""

    report = _run_b747_trim_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time"] == pytest.approx(60.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=99.0, maximum=102.0)
    require_bounded(history, "speed_m_s", minimum=150.0, maximum=155.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "mass_kg", minimum=288756.9, maximum=288756.9)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    assert independent_force_closure(history)["p99_normalized_residual"] < 1.0e-4
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 24_675_886.7,
            "inertia_y_kg_m2": 44_877_574.1,
            "inertia_z_kg_m2": 67_384_152.0,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 1.0e-4
####


def test_b747_generated_120_second_trim_hold_is_bounded(tmp_path: Path) -> None:
    """The generated B747 source-anchor hold remains steady for 120 seconds."""

    report = _run_b747_trim_120_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time"] == pytest.approx(120.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=99.0, maximum=102.0)
    require_bounded(history, "speed_m_s", minimum=150.0, maximum=155.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    assert independent_force_closure(history)["p99_normalized_residual"] < 1.0e-4
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 24_675_886.7,
            "inertia_y_kg_m2": 44_877_574.1,
            "inertia_z_kg_m2": 67_384_152.0,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 1.0e-4
####


def test_b747_generated_descent_recovery_120_second_maneuver_is_bounded(tmp_path: Path) -> None:
    """A native guidance transition descends, then recovers toward level flight."""

    report = _run_b747_descent_recovery_120_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
    descent = PhaseWindow("descent", 0.0, 60.0).select(history)
    recovery = PhaseWindow("recovery", 60.0, 120.0).select(history)
    require_net_change(descent, "altitude_m", "decreasing", minimum=700.0)
    require_net_change(recovery, "altitude_m", "increasing", minimum=150.0)
    require_bounded(history, "altitude_m", minimum=150.0, maximum=1_010.0)
    require_bounded(history, "speed_m_s", minimum=150.0, maximum=200.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "attitude_controller_saturated", minimum=0.0, maximum=1.0)
    assert independent_force_closure(history)["p99_normalized_residual"] < 1.0e-4
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 24_675_886.7,
            "inertia_y_kg_m2": 44_877_574.1,
            "inertia_z_kg_m2": 67_384_152.0,
        }
        for sample in history
    )
    # This controlled 20 ms maneuver has a measured finite-difference
    # rotational discretization envelope below 1e-3. The source-anchor trim
    # retains the stricter 1e-4 gate above.
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 1.0e-3
####


def test_b747_point_mass_reduction_uses_the_rigid_body_source_deck(tmp_path: Path) -> None:
    """The B747 reduction agrees with the 6DOF source query at the anchor."""

    reduction = _run_b747_reduction_3dof(tmp_path / "reduction")
    rigid_body = _run_b747_trim_6dof(tmp_path / "rigid")
    assert reduction.results and rigid_body.results
    point = reduction.results[0].states["1"][0].named
    rigid = rigid_body.results[0].states["1"][0].named
    q_s = rigid["aero_dynamic_pressure_pa"] * 510.96672
    assert point["cx"] == pytest.approx(rigid["aero_force_body_x_n"] / q_s, abs=2.0e-10)
    assert point["cy"] == pytest.approx(rigid["aero_force_body_y_n"] / q_s, abs=2.0e-10)
    assert point["cz"] == pytest.approx(rigid["aero_force_body_z_n"] / q_s, abs=2.0e-10)
    assert point["elevator-deg"] == pytest.approx(rigid["aero_query_elevator-deg"], abs=1.0e-12)
    assert point["thrust"] == pytest.approx(rigid["thrust_force_n"], rel=2.0e-4)
####


def test_b747_point_mass_and_rigid_body_short_histories_remain_in_parity(tmp_path: Path) -> None:
    """The B747 reduction tracks the rigid plant over the shared 100 ms window."""

    reduction = run_files(B747_REDUCTION_3DOF, B747_REDUCTION_TABLES, output_dir=tmp_path / "reduction", max_steps=20, profile=GrammarProfile.TAORYX)
    rigid_body = run_files(B747_PARITY_6DOF, B747_TRIM_TABLES, output_dir=tmp_path / "rigid", max_steps=20, profile=GrammarProfile.TAORYX)
    assert reduction.results and rigid_body.results
    point_history = reduction.results[0].states["1"]
    rigid_history = rigid_body.results[0].states["1"]
    assert len(point_history) == len(rigid_history)
    for point, rigid in zip(point_history, rigid_history, strict=True):
        assert point.time == pytest.approx(rigid.time, abs=1.0e-12)
        assert point.named["vel"] * 0.3048 == pytest.approx(rigid.named["speed_m_s"], abs=2.0e-2)
        # As with X8, this is the common prescribed-alpha phase only; it is not
        # a claim that a free 6DOF attitude remains identical to point mass.
        assert point.named["alt"] * 0.3048 == pytest.approx(rigid.named["altitude_m"], abs=6.0e-2)
    ####


def test_b747_long_6dof_trim_hold_converges_and_has_plant_report(tmp_path: Path) -> None:
    """The long B747 plant passes the common contract and convergence gates."""

    verification = verify_golden_plant(_trim_case(), tmp_path / "verification")
    assert verification.plant_golden
    assert verification.verdict == "plant-golden"
    convergence = verification.stages[10]
    assert convergence.status == "pass"
    assert convergence.evidence["factors"] == [1.0, 0.5, 0.25]
    assert convergence.evidence["channels"] == ["altitude_m", "speed_m_s", "aero_alpha_deg", "aero_sideslip_deg"]
####


def test_b747_long_6dof_approach_and_go_around_phases_are_ordered(tmp_path: Path) -> None:
    """The long 6DOF candidate executes approach then go-around in order."""

    report = _run_b747_approach_long_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time"] == pytest.approx(60.0, abs=1.0e-10)
    approach = PhaseWindow("approach", 0.0, 30.0).select(history)
    go_around = PhaseWindow("go-around", 30.0, 60.0).select(history)
    assert all(sample["_segment"] == pytest.approx(1.0) for sample in approach if sample["time"] < 29.99)
    assert all(sample["_segment"] == pytest.approx(2.0) for sample in go_around if sample["time"] > 30.0)
    for phase in (approach, go_around):
        require_bounded(phase, "altitude_m", minimum=99.0, maximum=102.0)
        require_bounded(phase, "speed_m_s", minimum=150.0, maximum=155.0)
        require_bounded(phase, "aero_alpha_deg", minimum=0.0, maximum=4.0)
        require_bounded(phase, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
####


def test_b747_long_6dof_approach_go_around_converges(tmp_path: Path) -> None:
    """The ordered approach/go-around endpoint is stable under refinement."""

    verification = verify_golden_plant(_approach_case(), tmp_path / "verification")
    assert verification.plant_golden
    assert verification.stages[10].status == "pass"
    assert verification.stages[10].evidence["factors"] == [1.0, 0.5, 0.25]
####


@pytest.mark.parametrize(
    ("label", "source", "replacement"),
    (
        ("speed-plus-one", "ydt=153.0096", "ydt=154.0096"),
        ("altitude-plus-one", "ecic x=6378237.0", "ecic x=6378238.0"),
    ),
)
def test_b747_long_6dof_trim_hold_accepts_bounded_perturbations(
    label: str,
    source: str,
    replacement: str,
    tmp_path: Path,
) -> None:
    """Small declared initial-state perturbations preserve the plant envelope."""

    perturbed = tmp_path / f"b747-{label}.prb"
    perturbed.write_text(B747_TRIM_6DOF.read_text(encoding="utf-8").replace(source, replacement), encoding="utf-8")
    report = run_files(
        perturbed,
        B747_TRIM_TABLES,
        output_dir=tmp_path / label,
        max_steps=4_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    history = _history(report)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    # A speed perturbation is intentionally not trim-balanced; accept its
    # bounded climb excursion separately from the nominal 100 m hold band.
    require_bounded(history, "altitude_m", minimum=98.0, maximum=130.0)
    require_bounded(history, "speed_m_s", minimum=149.0, maximum=156.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=4.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
####


@pytest.mark.artifact
def test_b747_long_candidate_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the long fixed-wing trajectory and generic Matplotlib views."""

    report = _run_b747(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-rectangle-3dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.latitude_deg",
                "taos.longitude_deg",
                "taos.mach",
                "taos.thrust_n",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_long_climb_cruise_writes_phase_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the long climb/cruise reduction and phase metrics."""

    report = _run_b747_climb_cruise_3dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-climb-cruise-3dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {"phases": [{"id": "climb-cruise", "start_s": 0.0, "end_s": 60.0, "acceptance": "altitude and downrange increase while Mach remains bounded"}]},
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=("taos.alt", "taos.vel", "taos.long", "taos.lat", "taos.mach"),
        )
    assert (destination / "phase-metrics.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_long_6dof_trim_hold_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the bounded 6DOF trim-hold evidence and standard plots."""

    report = _run_b747_trim_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-trim-hold-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.mach",
                "taos.aero_alpha_deg",
                "taos.aero_sideslip_deg",
                "taos.thrust_n",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_long_6dof_trim_hold_writes_contract_and_convergence_artifacts(
    artifact_dir: Path,
    tmp_path: Path,
) -> None:
    """Publish machine-readable plant, phase, and convergence evidence."""

    verification = verify_golden_plant(_trim_case(), tmp_path / "verification")
    destination = artifact_dir / "vehicle-family-validation" / "b747-trim-hold-contract"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(verification.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    convergence = verification.stages[10].evidence
    (destination / "convergence-report.json").write_text(
        json.dumps(convergence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "phases": [
                    {
                        "id": "source-anchor-trim-hold",
                        "start_s": 0.0,
                        "end_s": 60.0,
                        "acceptance": "altitude, speed, alpha, beta, mass, and equation residuals remain bounded",
                    }
                ]
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    assert verification.plant_golden
    assert (destination / "convergence-report.json").stat().st_size > 100
    assert (destination / "phase-metrics.json").stat().st_size > 100
####


@pytest.mark.artifact
def test_b747_long_6dof_approach_go_around_writes_phase_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the ordered approach and go-around phase evidence."""

    report = _run_b747_approach_long_6dof(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "b747-approach-go-around-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "phases": [
                    {"id": "approach", "start_s": 0.0, "end_s": 30.0, "segment": 1},
                    {"id": "go-around", "start_s": 30.0, "end_s": 60.0, "segment": 2},
                ]
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=(
                "taos.altitude_m",
                "taos.speed_m_s",
                "taos.aero_alpha_deg",
                "taos.aero_sideslip_deg",
                "taos.thrust_n",
            ),
        )
    assert (destination / "phase-metrics.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_b747_long_6dof_approach_go_around_writes_convergence_artifact(
    artifact_dir: Path,
    tmp_path: Path,
) -> None:
    """Publish approach/go-around convergence evidence."""

    verification = verify_golden_plant(_approach_case(), tmp_path / "verification")
    destination = artifact_dir / "vehicle-family-validation" / "b747-approach-go-around-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "convergence-report.json").write_text(
        json.dumps(verification.stages[10].evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    assert (destination / "convergence-report.json").stat().st_size > 100
####
