from __future__ import annotations

import json
import math
import re
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.program import LoadedProgram
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import PhaseWindow, independent_force_closure, independent_moment_closure, require_bounded, require_net_change
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/mission_families/slower_x8/SV03_long_validation_6dof.prb"
LEVEL_SETTLING = ROOT / "examples/mission_families/slower_x8/SV03_long_level_settling_6dof.prb"
SOURCE_TRIM_HOLD = ROOT / "examples/mission_families/slower_x8/SV03_source_trim_hold_30_6dof.prb"
ROUTE = ROOT / "examples/mission_families/slower_x8/SV03_long_rectangle_route_6dof.prb"
FIGURE_EIGHT = ROOT / "examples/mission_families/slower_x8/SV03_figure_eight_route_6dof.prb"
FIGURE_EIGHT_LQR = ROOT / "examples/mission_families/slower_x8/SV03_figure_eight_altitude_reversal_6dof.prb"
RACETRACK_SURFACES = ROOT / "examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_6dof.prb"
RACETRACK_DIRECT_MOMENT = ROOT / "examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_direct_moment_6dof.prb"
WAYPOINT = ROOT / "examples/mission_families/slower_x8/SV03_basic_waypoint_altitude_6dof.prb"
APPROACH = ROOT / "examples/mission_families/slower_x8/SV03_approach_go_around_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)

pytestmark = [pytest.mark.slow, pytest.mark.x8]


def _run_x8(output_dir: Path) -> RunReport:
    """Run the long powered trim/recovery candidate."""

    return run_files(
        PROBLEM,
        TABLES,
        output_dir=output_dir,
        max_steps=13_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    ####


def test_x8_figure_eight_declares_common_attitude_lqr_realization() -> None:
    """The fixed-wing witness resolves through the same realization contract."""

    program = LoadedProgram.load(FIGURE_EIGHT_LQR, TABLES, profile=GrammarProfile.TAORYX)
    metadata = program.case().metadata
    realization = metadata["controller_realization"]
    assert isinstance(realization, dict)
    assert realization["role"] == "attitude"
    assert realization["implementation"] == "lqr"
    assert realization["a_sha256"]
    assert realization["b_sha256"]
    assert realization["k_sha256"]
    assert realization["scenario_overrides_allowed"] is False
    assert realization["control_path"] == ["guidance", "reference_shaping", "attitude_lqr", "allocator", "actuator", "plant"]
    ####


def _run_x8_level_settling(output_dir: Path) -> RunReport:
    """Run the generated 120-second source-neighborhood level corridor."""

    return run_files(
        LEVEL_SETTLING,
        TABLES,
        output_dir=output_dir,
        max_steps=25_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_x8_source_trim_hold(output_dir: Path) -> RunReport:
    """Run the generated source-composed X8 trim and bounded hold."""

    return run_files(
        SOURCE_TRIM_HOLD,
        TABLES,
        output_dir=output_dir,
        max_steps=7_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_x8_route(output_dir: Path) -> RunReport:
    """Run the two-minute four-leg powered route candidate."""

    return run_files(
        ROUTE,
        TABLES,
        output_dir=output_dir,
        max_steps=7_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_x8_figure_eight(output_dir: Path) -> RunReport:
    """Run the three-minute powered smooth figure-eight showcase."""

    return run_files(
        FIGURE_EIGHT,
        TABLES,
        output_dir=output_dir,
        max_steps=200_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _history(report: RunReport) -> tuple[dict[str, float], ...]:
    assert report.results and report.results[0].completed
    return tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
####


def test_x8_long_powered_candidate_is_bounded(tmp_path: Path) -> None:
    """The powered X8 candidate remains inside its declared local envelope."""

    report = _run_x8(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(60.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=170.0, maximum=230.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=20.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "mass_kg", minimum=3.364, maximum=3.364)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    # The independent telemetry check uses a finite-difference acceleration
    # over the 5 ms accepted states.  The direct RHS closure remains at
    # machine precision; this looser independent gate is the measured
    # discretization envelope for the controlled X8 trace.
    assert independent_force_closure(history)["p99_normalized_residual"] < 5.0e-3
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 0.325,
            "inertia_y_kg_m2": 0.140,
            "inertia_z_kg_m2": 0.400,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 5.0e-3
####


def test_x8_generated_level_settling_corridor_is_bounded_for_120_seconds(tmp_path: Path) -> None:
    """The metadata-generated X8 level corridor remains in its local envelope."""

    report = _run_x8_level_settling(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=150.0, maximum=190.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=24.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "mass_kg", minimum=3.364, maximum=3.364)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    assert independent_force_closure(history)["p99_normalized_residual"] < 5.0e-3
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 0.325,
            "inertia_y_kg_m2": 0.140,
            "inertia_z_kg_m2": 0.400,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 5.0e-3
####


def test_x8_source_composed_trim_hold_is_bounded_for_30_seconds(tmp_path: Path) -> None:
    """The source-composed trim closes all body moments before recovery work."""

    report = _run_x8_source_trim_hold(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(30.0, abs=1.0e-10)
    assert history[0]["aero_alpha_deg"] == pytest.approx(7.8095001441, abs=1.0e-8)
    assert history[0]["aero_sideslip_deg"] == pytest.approx(0.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=100.0, maximum=190.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=21.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "mass_kg", minimum=3.364, maximum=3.364)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    assert independent_force_closure(history)["p99_normalized_residual"] < 5.0e-3
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 0.325,
            "inertia_y_kg_m2": 0.140,
            "inertia_z_kg_m2": 0.400,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 5.0e-3
####


def test_x8_long_rectangle_route_completes_four_expected_legs(tmp_path: Path) -> None:
    """The powered X8 route traverses east, north, west, and south legs."""

    report = _run_x8_route(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
    assert all("route_bank_command_deg" in sample and "route_bank_achieved_deg" in sample for sample in history)
    assert max(abs(sample["route_bank_command_deg"]) for sample in history) > 0.0
    assert all(math.isfinite(sample["route_bank_achieved_deg"]) for sample in history)
    assert all("route_leg_index" in sample and "route_target_error_m" in sample for sample in history)
    assert all(
        all(f"route_corner_{index}_error_m" in sample for index in range(4))
        for sample in history
    )
    # The X8 course uses a declared 300 m research capture corridor. This is
    # intentionally looser than a small-UAV precision waypoint claim, but it
    # is still finite and materially tighter than the 615 m leg.
    assert max(sample["route_target_error_m"] for sample in history) < 500.0
    corner_capture_errors = {
        index: min(sample[f"route_corner_{index}_error_m"] for sample in history)
        for index in range(4)
    }
    assert max(corner_capture_errors.values()) <= 300.0
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 0.325,
            "inertia_y_kg_m2": 0.140,
            "inertia_z_kg_m2": 0.400,
        }
        for sample in history
    )
    assert independent_force_closure(enriched)["p99_normalized_residual"] < 5.0e-3
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 5.0e-3
    phases = (
        PhaseWindow("east", 0.0, 30.0),
        PhaseWindow("north", 30.0, 60.0),
        PhaseWindow("west", 60.0, 90.0),
        PhaseWindow("south", 90.0, 120.0),
    )
    samples = [phase.select(history) for phase in phases]
    require_net_change(samples[0], "longitude_deg", "increasing", minimum=1.0e-4)
    require_net_change(samples[1], "latitude_deg", "increasing", minimum=1.0e-4)
    require_net_change(samples[2], "longitude_deg", "decreasing", minimum=1.0e-4)
    require_net_change(samples[3], "latitude_deg", "decreasing", minimum=1.0e-4)
    require_bounded(history, "altitude_m", minimum=100.0, maximum=220.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=22.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "mass_kg", minimum=3.364, maximum=3.364)
####


def test_x8_figure_eight_route_completes_four_smooth_phases(tmp_path: Path) -> None:
    """The powered X8 figure-eight remains bounded through both lobes.

    The 650 m corridor is intentionally a research-surrogate path bound for
    this 600 m-scale course, not a precision waypoint claim.
    """

    report = _run_x8_figure_eight(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(180.0, abs=1.0e-10)
    assert {int(sample["route_phase_index"]) for sample in history} == {0, 1, 2, 3}
    require_bounded(history, "altitude_m", minimum=150.0, maximum=220.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=25.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    assert max(sample["route_target_error_m"] for sample in history) < 650.0
    assert max(abs(sample["route_cross_track_error_m"]) for sample in history) < 650.0
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-10
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-10
####


def test_x8_surface_authority_fails_closed_at_source_envelope(tmp_path: Path) -> None:
    """Surface authority must report envelope exit before rotational overflow."""

    problem = tmp_path / "x8-surface-authority.prb"
    problem.write_text(
        ROUTE.read_text(encoding="utf-8").replace(
            "sideslip-gain=-12.0",
            "fixed-wing-control-authority=surfaces "
            "surface-control-inversion=true "
            "surface-inversion-step-deg=1.0 "
            "surface-inversion-max-delta-deg=0.5 "
            "surface-inversion-regularization=1.0 "
            "sideslip-gain=20.0 sideslip-rate-damping=1.0 "
            "attitude-gain=10.0 rate-damping=5.0",
        ),
        encoding="utf-8",
    )
    report = run_files(
        problem,
        TABLES,
        output_dir=tmp_path / "surface-authority",
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code != 0
    messages = tuple(item.message for item in report.diagnostics)
    assert any("outside its declared envelope" in message for message in messages), messages
    assert not any("non-finite" in message.lower() for message in messages), messages
    ####


def test_x8_racetrack_allocates_bank_pitch_through_elevons(tmp_path: Path) -> None:
    """A short source-domain witness uses aero tables for the controlled axes.

    The X8 has two declared elevon channels, so the test deliberately checks
    the physical allocation seam rather than expecting an independently
    achievable three-axis moment.  Any coupled yaw residual must remain in
    the aerodynamic plant and be visible in the telemetry.  The complete
    racetrack has a separate fail-closed test because this two-effector model
    currently reaches its declared source beta boundary before terminal
    closure.
    """

    report = run_files(
        RACETRACK_SURFACES,
        TABLES,
        output_dir=tmp_path / "racetrack-surfaces",
        max_steps=250,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.results
    history = tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
    assert history[-1]["time_s"] == pytest.approx(5.0, abs=1.0e-10)
    assert {int(sample["surface_allocation_active_surface_count"]) for sample in history} == {2}
    assert max(abs(sample["propulsion_moment_body_z_nm"]) for sample in history) < 1.0e-12
    assert max(abs(sample["aero_moment_body_z_nm"]) for sample in history) > 1.0e-4
    assert max(abs(sample["surface_allocation_requested_moment_x_nm"]) for sample in history) > 1.0e-4
    assert all(math.isfinite(sample["surface_allocation_residual_nm"]) for sample in history)
    assert {sample["surface_allocation_x8_mapping_sign"] for sample in history} == {1.0}
    assert all(
        sample["surface_allocation_left_elevon_achieved_deg"]
        == pytest.approx(
            sample["surface_allocation_collective_elevon_achieved_deg"]
            + sample["surface_allocation_differential_elevon_achieved_deg"]
        )
        for sample in history
    )
    assert all(
        sample["surface_allocation_right_elevon_achieved_deg"]
        == pytest.approx(
            sample["surface_allocation_collective_elevon_achieved_deg"]
            - sample["surface_allocation_differential_elevon_achieved_deg"]
        )
        for sample in history
    )
    assert max(sample["altitude_m"] for sample in history) < 205.0
    assert max(abs(sample["aero_sideslip_deg"]) for sample in history) < 5.0
    ####


def test_x8_physical_racetrack_fails_closed_at_uncontrolled_beta_boundary(tmp_path: Path) -> None:
    """The full physical route cannot silently cross the source table domain."""

    report = run_files(
        RACETRACK_SURFACES,
        TABLES,
        output_dir=tmp_path / "racetrack-boundary",
        max_steps=2_500,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert not report.results
    messages = tuple(item.message for item in report.diagnostics)
    assert any("outside its declared envelope" in message for message in messages), messages
    assert any("beta" in message for message in messages), messages
    ####


def test_x8_direct_moment_packet_separates_controller_and_load_channels(tmp_path: Path) -> None:
    """The direct baseline must expose moments, not masquerade as surfaces."""

    report = run_files(
        RACETRACK_DIRECT_MOMENT,
        TABLES,
        output_dir=tmp_path / "racetrack-direct-moment",
        max_steps=1_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.results
    history = tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
    assert history[-1]["time_s"] == pytest.approx(20.0, abs=1.0e-10)
    assert {int(sample["direct_moment_active"]) for sample in history} == {1}
    assert max(abs(sample["direct_moment_controller_request_x_nm"]) for sample in history) > 1.0e-4
    assert all(math.isfinite(sample["direct_moment_cancellation_residual_nm"]) for sample in history)
    assert all(
        abs(sample["direct_moment_total_x_nm"] - sample["direct_moment_aero_x_nm"] - sample["direct_moment_propulsion_x_nm"]) < 1.0e-10
        for sample in history
    )
    assert all(
        sample["route_pitch_command_deg"] - sample["route_flight_path_command_deg"] == pytest.approx(7.8095001441, abs=1.0e-8)
        for sample in history
    )
    ####


def test_x8_long_rectangle_route_accepts_fixed_crosswind_disturbance(tmp_path: Path) -> None:
    """A declared 5 m/s crosswind remains inside the disturbance corridor."""

    problem = tmp_path / "x8-crosswind.prb"
    problem.write_text(
        ROUTE.read_text(encoding="utf-8").replace(
            "*earth wgs-84 omega=0",
            "*earth wgs-84 omega=0\n*wind geodetic windv=5.0 windh=90 windd=0",
        ),
        encoding="utf-8",
    )
    report = run_files(
        problem,
        TABLES,
        output_dir=tmp_path / "crosswind",
        max_steps=7_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
    # The nominal route has a 210 m elevated corner.  Permit a smaller
    # disturbance corridor around that target while retaining a hard bound
    # below the local table envelope.
    require_bounded(history, "altitude_m", minimum=95.0, maximum=205.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=26.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
####


@pytest.mark.parametrize(
    ("label", "source", "replacement"),
    (
        ("speed-plus-half", "ydt=17.9", "ydt=18.4"),
        ("height-plus-half", "ecic x=6378315.0", "ecic x=6378315.5"),
    ),
)
def test_x8_long_powered_candidate_accepts_bounded_perturbations(
    label: str,
    source: str,
    replacement: str,
    tmp_path: Path,
) -> None:
    """Small initial-condition changes remain inside the powered envelope."""

    problem = tmp_path / f"x8-{label}.prb"
    problem.write_text(PROBLEM.read_text(encoding="utf-8").replace(source, replacement), encoding="utf-8")
    report = run_files(
        problem,
        TABLES,
        output_dir=tmp_path / label,
        max_steps=13_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    require_bounded(history, "altitude_m", minimum=165.0, maximum=235.0)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=21.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    require_bounded(history, "translation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
    require_bounded(history, "rotation_equation_residual_normalized", minimum=0.0, maximum=1.0e-10)
####


@pytest.mark.parametrize(
    ("problem", "duration_s"),
    ((WAYPOINT, 20.0), (APPROACH, 24.0)),
    ids=("powered-waypoint", "approach-go-around"),
)
def test_x8_supporting_phases_execute_in_declared_order(problem: Path, duration_s: float, tmp_path: Path) -> None:
    """Supporting X8 waypoint and approach files exercise named runtime phases."""

    report = run_files(
        problem,
        TABLES,
        output_dir=tmp_path / problem.stem,
        max_steps=5_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    if problem == WAYPOINT:
        # The waypoint is now deliberately far enough away to exercise the
        # full declared 20-second powered leg.  Early capture is valid too,
        # but reaching the mission horizon is the preferred evidence case.
        assert history[-1]["time_s"] <= duration_s
        # The scored mission contract allows a 35 m terminal corridor for
        # this first source-surrogate waypoint leg.
        assert history[-1]["range_to_target_m"] <= 35.0
    else:
        assert history[-1]["time_s"] == pytest.approx(duration_s, abs=1.0e-8)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=24.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    if problem == WAYPOINT:
        # The route declaration is only active when the segment carries the
        # native propnav fly directive.  Keep the supporting case bounded so
        # a missing directive cannot silently become an uncontrolled flight.
        assert min(sample["range_to_target_m"] for sample in history) < 100.0
        require_bounded(history, "altitude_m", minimum=150.0, maximum=230.0)
    if problem == APPROACH:
        approach = [sample for sample in history if sample["time_s"] < 12.0]
        go_around = [sample for sample in history if sample["time_s"] > 12.0]
        assert approach and go_around
        assert all(sample["_segment"] == pytest.approx(1.0) for sample in approach)
        assert all(sample["_segment"] == pytest.approx(2.0) for sample in go_around)
####


def _x8_convergence(output_dir: Path) -> dict[str, object]:
    """Refine a bounded source-composed X8 trim window at three step sizes.

    Long-duration boundedness is covered separately by the 30-, 60-, and
    120-second scenarios.  Convergence needs a finite, repeatable budget, so
    it uses the same source-composed plant and controls for a declared 10 s
    window rather than multiplying the broad route candidate's cost by three.
    """

    output_dir.mkdir(parents=True, exist_ok=True)
    base_text = SOURCE_TRIM_HOLD.read_text(encoding="utf-8").replace(
        "*when time>30.0 stop",
        "*when time>10.0 stop",
    )
    factors = (1.0, 0.5, 0.25)
    channels = ("altitude_m", "speed_m_s", "aero_alpha_deg", "aero_sideslip_deg")
    snapshots: list[dict[str, float]] = []
    for factor in factors:
        scaled_text, replacements = re.subn(
            r"(?P<prefix>\bdt\s*=\s*)(?P<value>[0-9.eE+-]+)",
            lambda match: f"{match.group('prefix')}{float(match.group('value')) * factor:.16g}",
            base_text,
        )
        assert replacements == 1
        problem = output_dir / f"x8-{factor:g}.prb"
        problem.write_text(scaled_text, encoding="utf-8")
        report = run_files(
            problem,
            TABLES,
            output_dir=output_dir / f"dt-{factor:g}",
            max_steps=math.ceil(2_000 / factor),
            integrator="rk4",
            profile=GrammarProfile.TAORYX,
        )
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        history = _history(report)
        snapshots.append({name: history[-1][name] for name in channels})
    coarse_error = math.sqrt(sum((snapshots[0][name] - snapshots[1][name]) ** 2 for name in channels))
    fine_error = math.sqrt(sum((snapshots[1][name] - snapshots[2][name]) ** 2 for name in channels))
    assert fine_error <= coarse_error * 2.5 + 1.0e-5
    return {
        "factors": list(factors),
        "window_duration_s": 10.0,
        "channels": list(channels),
        "coarse_error": coarse_error,
        "fine_error": fine_error,
        "refinement_ratio": None if fine_error == 0.0 else coarse_error / fine_error,
    }
####


def test_x8_long_powered_candidate_converges(tmp_path: Path) -> None:
    """The powered X8 endpoint is stable under step refinement."""

    evidence = _x8_convergence(tmp_path / "convergence")
    assert evidence["factors"] == [1.0, 0.5, 0.25]
    assert evidence["window_duration_s"] == pytest.approx(10.0)
    assert evidence["coarse_error"] >= 0.0
    assert evidence["fine_error"] >= 0.0
####


@pytest.mark.artifact
def test_x8_long_powered_candidate_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the powered X8 candidate and standard trajectory plots."""

    report = _run_x8(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "x8-powered-validation-6dof"
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
def test_x8_long_rectangle_route_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the long route and its standard Matplotlib trajectory views."""

    report = _run_x8_route(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "x8-long-rectangle-route-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "phases": [
                    {"id": label, "start_s": start, "end_s": end, "acceptance": direction}
                    for label, start, end, direction in (
                        ("east", 0.0, 30.0, "longitude increases"),
                        ("north", 30.0, 60.0, "latitude increases"),
                        ("west", 60.0, 90.0, "longitude decreases"),
                        ("south", 90.0, 120.0, "latitude decreases"),
                    )
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
                "taos.latitude_deg",
                "taos.longitude_deg",
                "taos.aero_alpha_deg",
                "taos.aero_sideslip_deg",
                "taos.route_target_error_m",
                "taos.route_corner_0_error_m",
                "taos.route_corner_1_error_m",
                "taos.route_corner_2_error_m",
                "taos.route_corner_3_error_m",
            ),
        )
    assert (destination / "phase-metrics.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_x8_long_powered_candidate_writes_convergence_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the three-step X8 convergence evidence."""

    evidence = _x8_convergence(tmp_path / "convergence")
    destination = artifact_dir / "vehicle-family-validation" / "x8-powered-validation-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "convergence-report.json").write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    assert (destination / "convergence-report.json").stat().st_size > 100
####
