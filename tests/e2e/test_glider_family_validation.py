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
PROBLEM = ROOT / "examples/mission_families/hypersonic_glide_terminal_guidance/mission.prb"
TABLES = (ROOT / "examples/mission_families/hypersonic_glide_terminal_guidance/aero.tbl",)
GLIDER_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/long_unpowered_glide_6dof.prb"
GLIDER_6DOF_TABLES = (ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl",)

pytestmark = pytest.mark.slow


def _run_glider(output_dir: Path) -> RunReport:
    """Run the executable legacy-shaped glider extension scenario."""

    return run_files(
        PROBLEM,
        TABLES,
        output_dir=output_dir,
        max_steps=5_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _run_glider_6dof(problem: Path, output_dir: Path, *, max_steps: int = 3_000) -> RunReport:
    """Run the source-anchored unpowered rigid-body glider."""

    return run_files(
        problem,
        GLIDER_6DOF_TABLES,
        output_dir=output_dir,
        max_steps=max_steps,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
####


def _history(report: RunReport) -> tuple[dict[str, float], ...]:
    assert report.results and report.results[0].completed
    return tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
####


def test_glider_extension_candidate_has_release_and_terminal_phases(tmp_path: Path) -> None:
    """The current 3DOF glider path remains unpowered and loses energy."""

    report = _run_glider(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(59.9, abs=0.2)
    release = PhaseWindow("release-coast", 0.0, 30.0).select(history)
    terminal = PhaseWindow("terminal-guidance", 30.0, 59.9).select(history)
    assert all(sample["thrust"] == pytest.approx(0.0) for sample in history)
    assert all(sample["mdot"] == pytest.approx(0.0) for sample in history)
    assert all(sample["mass"] == pytest.approx(500.0) for sample in history)
    require_net_change(release, "alt", "decreasing", minimum=1.0)
    require_net_change(terminal, "alt", "decreasing", minimum=1.0)
    require_net_change(history, "long", "increasing", minimum=0.5)
    require_bounded(history, "mach", minimum=0.0, maximum=8.0)
    require_bounded(history, "gama", minimum=-90.0, maximum=90.0)
    require_bounded(history, "mass", minimum=500.0, maximum=500.0)
####


def test_glider_6dof_long_release_energy_management_and_terminal_descent(tmp_path: Path) -> None:
    """The source-anchored glider trades energy and reaches terminal descent."""

    report = _run_glider_6dof(GLIDER_6DOF, tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(120.0, abs=1.0e-10)
    energy_management = PhaseWindow("energy-management", 0.0, 30.0).select(history)
    terminal = PhaseWindow("terminal-descent", 60.0, 120.0).select(history)
    require_net_change(energy_management, "altitude_m", "increasing", minimum=500.0)
    require_net_change(terminal, "altitude_m", "decreasing", minimum=5_000.0)
    require_net_change(history, "range_to_target_m", "decreasing", minimum=100_000.0)
    require_bounded(history, "altitude_m", minimum=0.0, maximum=30_000.0)
    require_bounded(history, "mach", minimum=1.0, maximum=6.7)
    require_bounded(history, "aero_alpha_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "mass_kg", minimum=14641.0545, maximum=14641.0545)
    require_bounded(history, "thrust_n", minimum=0.0, maximum=0.0)
    require_bounded(history, "propellant_mass_kg", minimum=0.0, maximum=0.0)
####


@pytest.mark.parametrize(
    ("label", "source", "replacement"),
    (
        ("release-speed-plus-one", "ydt=680.596575558", "ydt=681.596575558"),
        ("release-height-plus-one", "z=3608628.71159", "z=3608629.71159"),
    ),
)
def test_glider_6dof_accepts_small_release_perturbations(
    label: str,
    source: str,
    replacement: str,
    tmp_path: Path,
) -> None:
    """Small release-state changes preserve the unpowered envelope."""

    problem = tmp_path / f"glider-{label}.prb"
    problem.write_text(GLIDER_6DOF.read_text(encoding="utf-8").replace(source, replacement), encoding="utf-8")
    report = _run_glider_6dof(problem, tmp_path / label)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    require_bounded(history, "altitude_m", minimum=0.0, maximum=30_000.0)
    require_bounded(history, "mach", minimum=1.0, maximum=6.7)
    require_bounded(history, "aero_alpha_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
####


def _glider_6dof_convergence(output_dir: Path) -> dict[str, object]:
    """Compare terminal glider snapshots under timestep refinement."""

    output_dir.mkdir(parents=True, exist_ok=True)
    source = GLIDER_6DOF.read_text(encoding="utf-8")
    factors = (1.0, 0.5, 0.25)
    channels = ("altitude_m", "speed_m_s", "mach", "aero_alpha_deg", "aero_sideslip_deg")
    snapshots: list[dict[str, float]] = []
    for factor in factors:
        scaled, replacements = re.subn(
            r"(?P<prefix>\bdt\s*=\s*)(?P<value>[0-9.eE+-]+)",
            lambda match: f"{match.group('prefix')}{float(match.group('value')) * factor:.16g}",
            source,
        )
        assert replacements == 1
        problem = output_dir / f"glider-{factor:g}.prb"
        problem.write_text(scaled, encoding="utf-8")
        report = _run_glider_6dof(problem, output_dir / f"dt-{factor:g}", max_steps=math.ceil(3_000 / factor))
        assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
        history = _history(report)
        snapshots.append({name: history[-1][name] for name in channels})
    coarse = math.sqrt(sum((snapshots[0][name] - snapshots[1][name]) ** 2 for name in channels))
    fine = math.sqrt(sum((snapshots[1][name] - snapshots[2][name]) ** 2 for name in channels))
    assert fine <= coarse * 2.5 + 1.0e-5
    return {"factors": list(factors), "channels": list(channels), "coarse_error": coarse, "fine_error": fine}
####


def test_glider_6dof_converges_under_timestep_refinement(tmp_path: Path) -> None:
    """The terminal glider endpoint remains stable as the step is refined."""

    evidence = _glider_6dof_convergence(tmp_path / "convergence")
    assert evidence["factors"] == [1.0, 0.5, 0.25]
####


@pytest.mark.artifact
def test_glider_extension_candidate_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish the current glider extension evidence and standard plots."""

    report = _run_glider(tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "glider-3dof-extension"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(
        json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "verdict": "blocked",
                "reason": "3DOF extension evidence only; dedicated 6DOF glider contract remains missing",
                "phases": [
                    {"id": "release-coast", "start_s": 0.0, "end_s": 30.0},
                    {"id": "terminal-guidance", "start_s": 30.0, "end_s": 59.9},
                ],
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
            channels=("taos.alt", "taos.vel", "taos.mach", "taos.gama", "taos.east"),
        )
    assert (destination / "phase-metrics.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####


@pytest.mark.artifact
def test_glider_6dof_writes_family_artifacts(artifact_dir: Path, tmp_path: Path) -> None:
    """Publish long glider run, phase, convergence, and Matplotlib evidence."""

    report = _run_glider_6dof(GLIDER_6DOF, tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    destination = artifact_dir / "vehicle-family-validation" / "glider-unpowered-6dof"
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "run-report.json").write_text(json.dumps(report.as_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (destination / "phase-metrics.json").write_text(
        json.dumps(
            {
                "verdict": "candidate",
                "phases": [
                    {"id": "release-energy-management", "start_s": 0.0, "end_s": 30.0},
                    {"id": "terminal-descent", "start_s": 60.0, "end_s": 120.0},
                ],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    convergence = _glider_6dof_convergence(tmp_path / "convergence")
    (destination / "convergence-report.json").write_text(json.dumps(convergence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    for artifact in report.artifacts:
        render_run_artifact_plots(
            artifact,
            destination / "plots",
            vehicle_id="1",
            channels=("taos.altitude_m", "taos.speed_m_s", "taos.mach", "taos.aero_alpha_deg", "taos.aero_sideslip_deg", "taos.range_to_target_m"),
        )
    assert (destination / "convergence-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####
