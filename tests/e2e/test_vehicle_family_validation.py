from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import PhaseWindow, require_bounded, require_net_change
from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, verify_golden_plant

ROOT = Path(__file__).resolve().parents[2]
B747_PROBLEM = ROOT / "examples/mission_families/slower_b747/SV01_rectangle_course_3dof.prb"
B747_TRIM_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_long_trim_hold_6dof.prb"
B747_APPROACH_LONG_6DOF = ROOT / "examples/mission_families/slower_b747/SV01_approach_go_around_long_6dof.prb"
B747_CLIMB_CRUISE_3DOF = ROOT / "examples/mission_families/slower_b747/SV01_climb_cruise_3dof.prb"
B747_TABLES = (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_static_6axis.tbl",
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_jt9d_thrust.tbl",
)
B747_TRIM_TABLES = (
    ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables/b747_nominal_elevator_6axis.tbl",
)

pytestmark = pytest.mark.slow


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
