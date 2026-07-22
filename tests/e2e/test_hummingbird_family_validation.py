from __future__ import annotations

import json
import math
import re
from pathlib import Path

import pytest

from taoryx.language import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import PhaseWindow, independent_force_closure, independent_moment_closure, require_bounded, require_net_change
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[2]
TABLES = tuple(
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables" / name
    for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    )
)
HOVER = ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb"
DISTURBED = ROOT / "examples/mission_families/slower_hummingbird/SV05_disturbed_response_6dof.prb"
RECTANGLE = ROOT / "examples/mission_families/slower_hummingbird/SV05_rectangle_course_6dof.prb"
TAKEOFF = ROOT / "examples/mission_families/slower_hummingbird/SV05_takeoff_6dof.prb"
LANDING = ROOT / "examples/mission_families/slower_hummingbird/SV05_landing_6dof.prb"
RETURN_HOME_LAND = ROOT / "examples/mission_families/slower_hummingbird/SV05_return_home_land_6dof.prb"

pytestmark = [pytest.mark.slow, pytest.mark.dof6, pytest.mark.hummingbird]


def _run(problem: Path, output_dir: Path, max_steps: int) -> RunReport:
    """Run one Hummingbird problem through normal TAORYX file ingestion."""

    return run_files(problem, TABLES, output_dir=output_dir, max_steps=max_steps, profile=GrammarProfile.TAORYX)
####


def _history(report: RunReport) -> tuple[dict[str, float], ...]:
    assert report.results and report.results[0].completed
    return tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
####


def _require_finite(history: tuple[dict[str, float], ...], *channels: str) -> None:
    for sample in history:
        for channel in channels:
            assert channel in sample, channel
            assert sample[channel] == pytest.approx(sample[channel]), (channel, sample[channel])
####


def test_hummingbird_rectangle_route_has_expected_phase_displacements(tmp_path: Path) -> None:
    """The 40-second route reaches the elevated corner and returns home."""

    report = _run(RECTANGLE, tmp_path, max_steps=50_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(40.0, abs=1.0e-10)
    _require_finite(history, "altitude_m", "latitude_deg", "longitude_deg", "speed_m_s", "aero_query_rotor_speed")
    phases = (
        PhaseWindow("hover-leg", 0.0, 10.0),
        PhaseWindow("elevated-corner", 10.0, 20.0),
        PhaseWindow("return-leg", 20.0, 30.0),
        PhaseWindow("landing-checkpoint", 30.0, 40.0),
    )
    windows = [phase.select(history) for phase in phases]
    require_net_change(windows[0], "longitude_deg", "increasing", minimum=1.0e-6)
    require_net_change(windows[1], "latitude_deg", "increasing", minimum=1.0e-6)
    require_net_change(windows[1], "altitude_m", "increasing", minimum=0.2)
    require_net_change(windows[2], "longitude_deg", "decreasing", minimum=1.0e-6)
    require_net_change(windows[3], "latitude_deg", "decreasing", minimum=1.0e-6)
    require_bounded(history, "altitude_m", minimum=1.8, maximum=3.2)
    require_bounded(history, "speed_m_s", minimum=0.0, maximum=1.0)
    require_bounded(history, "aero_query_rotor_speed", minimum=0.0, maximum=1500.0)
    assert abs(history[-1]["latitude_deg"]) < 1.0e-6
    assert abs(history[-1]["longitude_deg"]) < 1.0e-6
    assert max(abs(sample[name]) for sample in history for name in ("wx", "wy", "wz")) < 0.1
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert independent_force_closure(history)["p99_normalized_residual"] < 1.0e-6
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 0.00365,
            "inertia_y_kg_m2": 0.00368,
            "inertia_z_kg_m2": 0.00703,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 1.0e-6
####


def test_hummingbird_return_home_landing_orders_touchdown_before_shutdown(tmp_path: Path) -> None:
    """The combined return phase reaches home before touchdown and shutdown."""

    report = _run(RETURN_HOME_LAND, tmp_path, max_steps=25_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert 18.0 <= history[-1]["time_s"] <= 20.0
    _require_finite(history, "range_to_target_m", "altitude_m", "speed_m_s", "motor_shutdown")
    return_phase = PhaseWindow("return-home", 0.0, 9.0).select(history)
    landing_phase = PhaseWindow("landing", 9.0, 18.5).select(history)
    require_net_change(return_phase, "range_to_target_m", "decreasing", minimum=0.5)
    require_net_change(landing_phase, "altitude_m", "decreasing", minimum=0.01)
    require_bounded(history, "altitude_m", minimum=0.0, maximum=2.1)
    require_bounded(history, "speed_m_s", minimum=0.0, maximum=1.0)
    require_bounded(history, "aero_query_rotor_speed", minimum=0.0, maximum=1500.0)
    assert history[-1]["range_to_target_m"] < 1.0e-3
    assert history[-1]["altitude_m"] < 1.0e-3
    assert history[-1]["motor_shutdown"] == pytest.approx(1.0)
    assert all(sample["motor_shutdown"] == pytest.approx(0.0) for sample in history if sample["time_s"] < 18.0)
####


@pytest.mark.parametrize(
    ("label", "source", "replacement"),
    (
        ("offset-plus-ten-cm", "y=0.5", "y=0.6"),
        ("altitude-plus-ten-cm", "ecic x=6378139.0", "ecic x=6378139.1"),
    ),
)
def test_hummingbird_return_home_landing_accepts_bounded_perturbations(
    label: str,
    source: str,
    replacement: str,
    tmp_path: Path,
) -> None:
    """Small return/landing initial-state perturbations remain recoverable."""

    problem = tmp_path / f"hummingbird-{label}.prb"
    problem.write_text(RETURN_HOME_LAND.read_text(encoding="utf-8").replace(source, replacement), encoding="utf-8")
    report = _run(problem, tmp_path / label, max_steps=25_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    _require_finite(history, "range_to_target_m", "altitude_m", "speed_m_s", "motor_shutdown")
    require_bounded(history, "altitude_m", minimum=0.0, maximum=2.3)
    require_bounded(history, "speed_m_s", minimum=0.0, maximum=1.0)
    require_bounded(history, "aero_query_rotor_speed", minimum=0.0, maximum=1500.0)
    assert history[-1]["range_to_target_m"] < 1.0e-3
    assert history[-1]["altitude_m"] < 1.0e-3
    assert history[-1]["motor_shutdown"] == pytest.approx(1.0)
####


def _return_home_convergence(output_dir: Path) -> dict[str, object]:
    """Run the combined sequence at three integration-step sizes."""

    output_dir.mkdir(parents=True, exist_ok=True)
    base_text = RETURN_HOME_LAND.read_text(encoding="utf-8")
    snapshots: list[dict[str, float]] = []
    factors = (1.0, 0.5, 0.25)
    for factor in factors:
        scaled_text, replacements = re.subn(
            r"(?P<prefix>\bdt\s*=\s*)(?P<value>[0-9.eE+-]+)",
            lambda match: f"{match.group('prefix')}{float(match.group('value')) * factor:.16g}",
            base_text,
        )
        assert replacements == 1
        problem = output_dir / f"return-home-{factor:g}.prb"
        problem.write_text(scaled_text, encoding="utf-8")
        report = _run(problem, output_dir / f"dt-{factor:g}", max_steps=math.ceil(25_000 / factor))
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        history = _history(report)
        final = history[-1]
        snapshots.append({name: final[name] for name in ("range_to_target_m", "altitude_m", "speed_m_s")})
    channels = ("range_to_target_m", "altitude_m", "speed_m_s")
    coarse_error = math.sqrt(sum((snapshots[0][name] - snapshots[1][name]) ** 2 for name in channels))
    fine_error = math.sqrt(sum((snapshots[1][name] - snapshots[2][name]) ** 2 for name in channels))
    assert fine_error <= coarse_error * 2.5 + 1.0e-5
    return {
        "factors": list(factors),
        "channels": list(channels),
        "coarse_error": coarse_error,
        "fine_error": fine_error,
        "refinement_ratio": None if fine_error == 0.0 else coarse_error / fine_error,
    }
####


def test_hummingbird_return_home_landing_converges(tmp_path: Path) -> None:
    """The combined return/landing endpoint is stable under step refinement."""

    evidence = _return_home_convergence(tmp_path)
    assert evidence["factors"] == [1.0, 0.5, 0.25]
    assert evidence["coarse_error"] >= 0.0
    assert evidence["fine_error"] >= 0.0
####


@pytest.mark.artifact
def test_hummingbird_rectangle_route_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish route, altitude, speed, and attitude plots for review."""

    report = _run(RECTANGLE, tmp_path, max_steps=50_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "hummingbird-rectangle"
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
                "taos.aero_query_rotor_speed",
                "taos.wx",
                "taos.wy",
                "taos.wz",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


def test_hummingbird_symmetric_hover_has_bounded_attitude_and_rotor_state(tmp_path: Path) -> None:
    """The open-loop hover plant remains finite for a meaningful dwell."""

    report = _run(HOVER, tmp_path, max_steps=2_100)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(10.0, abs=1.0e-10)
    _require_finite(history, "qw", "qx", "qy", "qz", "wx", "wy", "wz", "mass_kg", "aero_query_rotor_speed")
    assert all(sample["mass_kg"] == pytest.approx(0.5) for sample in history)
    assert all(0.0 <= sample["aero_query_rotor_speed"] <= 1500.0 for sample in history)
    assert max(abs(sample["wx"]) + abs(sample["wy"]) + abs(sample["wz"]) for sample in history) < 12.57
    assert max(abs(sample["altitude_m"] - history[0]["altitude_m"]) for sample in history) < 0.5
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-8
####


def test_hummingbird_disturbance_response_remains_inside_plant_envelope(tmp_path: Path) -> None:
    """An initial translational disturbance remains bounded for five seconds."""

    report = _run(DISTURBED, tmp_path, max_steps=2_100)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(5.0, abs=1.0e-10)
    _require_finite(history, "speed_m_s", "aero_airspeed_m_s", "aero_dynamic_pressure_pa", "mass_kg")
    assert max(sample["speed_m_s"] for sample in history) < 10.0
    assert all(sample["mass_kg"] == pytest.approx(0.5) for sample in history)
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-8
####


@pytest.mark.parametrize(
    ("problem", "direction", "minimum_altitude_m", "maximum_altitude_m", "final_altitude_m"),
    (
        (TAKEOFF, "increasing", 0.0, 2.5, 1.8),
        (LANDING, "decreasing", 0.0, 2.2, 0.1),
    ),
    ids=("takeoff", "landing-altitude-checkpoint"),
)
def test_hummingbird_altitude_phase_reaches_declared_checkpoint(
    problem: Path,
    direction: str,
    minimum_altitude_m: float,
    maximum_altitude_m: float,
    final_altitude_m: float,
    tmp_path: Path,
) -> None:
    """Takeoff and landing files move altitude in the declared direction."""

    report = _run(problem, tmp_path / problem.stem, max_steps=11_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    if direction == "increasing":
        assert history[-1]["time_s"] == pytest.approx(10.0, abs=1.0e-10)
    else:
        assert 9.0 <= history[-1]["time_s"] <= 10.0
    _require_finite(history, "altitude_m", "speed_m_s", "aero_query_rotor_speed", "mass_kg", "motor_shutdown")
    require_net_change(history, "altitude_m", direction, minimum=final_altitude_m / 2.0)
    require_bounded(history, "altitude_m", minimum=minimum_altitude_m, maximum=maximum_altitude_m)
    require_bounded(history, "speed_m_s", minimum=0.0, maximum=1.0)
    require_bounded(history, "aero_query_rotor_speed", minimum=0.0, maximum=1500.0)
    assert history[-1]["altitude_m"] >= final_altitude_m if direction == "increasing" else history[-1]["altitude_m"] <= final_altitude_m
    if direction == "decreasing":
        assert history[-1]["motor_shutdown"] == pytest.approx(1.0)
        assert all(sample["motor_shutdown"] == pytest.approx(0.0) for sample in history if sample["time_s"] < 9.0)
    assert max(sample["translation_equation_residual_normalized"] for sample in history) < 1.0e-8
    assert max(sample["rotation_equation_residual_normalized"] for sample in history) < 1.0e-8
####


@pytest.mark.artifact
@pytest.mark.parametrize("problem", (TAKEOFF, LANDING), ids=("takeoff", "landing"))
def test_hummingbird_altitude_phases_write_artifacts(problem: Path, artifact_dir: Path, tmp_path: Path) -> None:
    """Publish takeoff and landing altitude checkpoint plots."""

    report = _run(problem, tmp_path / problem.stem, max_steps=11_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / problem.stem
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
            channels=("taos.altitude_m", "taos.speed_m_s", "taos.aero_query_rotor_speed", "taos.wx", "taos.wy", "taos.wz"),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_hummingbird_return_home_landing_writes_phase_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the combined return, touchdown, and shutdown evidence."""

    report = _run(RETURN_HOME_LAND, tmp_path, max_steps=25_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "hummingbird-return-home-land"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "phases": [
                    {"id": "return-home", "start_s": 0.0, "end_s": 9.0, "metric": "range_to_target_m decreases"},
                    {"id": "landing", "start_s": 9.0, "end_s": 18.5, "metric": "altitude_m decreases"},
                    {"id": "motor-shutdown", "event_s": 18.0, "metric": "motor_shutdown becomes 1"},
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
                "taos.range_to_target_m",
                "taos.aero_query_rotor_speed",
                "taos.motor_shutdown",
            ),
        )
    assert (destination / "phase-metrics.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_hummingbird_return_home_landing_writes_convergence_artifact(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the three-step numerical convergence evidence."""

    evidence = _return_home_convergence(tmp_path / "convergence")
    destination = artifact_dir / "vehicle-family-validation" / "hummingbird-return-home-land"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "convergence-report.json").write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    assert (destination / "convergence-report.json").stat().st_size > 100
####


@pytest.mark.artifact
def test_hummingbird_hover_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the hover trace and standard Matplotlib family plots."""

    report = _run(HOVER, tmp_path, max_steps=2_100)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "hummingbird-hover"
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
                "taos.aero_dynamic_pressure_pa",
                "taos.aero_force_body_z_n",
                "taos.wx",
                "taos.wy",
                "taos.wz",
            ),
        )
    assert (destination / "run-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####
