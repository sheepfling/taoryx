from __future__ import annotations

import json
import math
import re
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import PhaseWindow, require_bounded, require_net_change
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/mission_families/slower_x8/SV03_long_validation_6dof.prb"
ROUTE = ROOT / "examples/mission_families/slower_x8/SV03_long_rectangle_route_6dof.prb"
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

pytestmark = pytest.mark.slow


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
####


def test_x8_long_rectangle_route_completes_four_expected_legs(tmp_path: Path) -> None:
    """The powered X8 route traverses east, north, west, and south legs."""

    report = _run_x8_route(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
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


def test_x8_long_rectangle_route_rejects_fixed_crosswind_disturbance(tmp_path: Path) -> None:
    """A declared 5 m/s crosswind keeps the powered route inside its envelope."""

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
    require_bounded(history, "altitude_m", minimum=95.0, maximum=185.0)
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
    assert history[-1]["time_s"] == pytest.approx(duration_s, abs=1.0e-8)
    require_bounded(history, "speed_m_s", minimum=15.0, maximum=21.0)
    require_bounded(history, "aero_alpha_deg", minimum=0.0, maximum=12.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-5.0, maximum=5.0)
    if problem == APPROACH:
        approach = [sample for sample in history if sample["time_s"] < 12.0]
        go_around = [sample for sample in history if sample["time_s"] > 12.0]
        assert approach and go_around
        assert all(sample["_segment"] == pytest.approx(1.0) for sample in approach)
        assert all(sample["_segment"] == pytest.approx(2.0) for sample in go_around)
####


def _x8_convergence(output_dir: Path) -> dict[str, object]:
    """Run the powered X8 candidate at three integration-step sizes."""

    output_dir.mkdir(parents=True, exist_ok=True)
    base_text = PROBLEM.read_text(encoding="utf-8")
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
            max_steps=math.ceil(13_000 / factor),
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
