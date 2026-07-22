from __future__ import annotations

import json
import math
import re
from pathlib import Path

import pytest

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.validation import PhaseWindow, independent_force_closure, independent_moment_closure, require_bounded, require_net_change, specific_energy
from taoryx.visualization import render_run_artifact_plots

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/mission_families/hypersonic_glide_terminal_guidance/mission.prb"
TABLES = (ROOT / "examples/mission_families/hypersonic_glide_terminal_guidance/aero.tbl",)
GLIDER_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/long_unpowered_glide_6dof.prb"
GLIDER_ROUTE_GEOMETRY_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/route_geometry_diagnostic_6dof.prb"
GLIDER_6DOF_TABLES = (ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables/x15_static_6axis.tbl",)
GLIDER_BANK_REVERSAL_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/long_unpowered_glide_bank_reversal_6dof.prb"
GLIDER_TRIM_HOLD_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/source_trim_hold_6dof.prb"
GLIDER_TRIM_THEN_PROPNAV_6DOF = ROOT / "examples/showcases/x15_rocket_to_hawaii/source_trim_then_propnav_6dof.prb"
X15_SURFACE_HOLD_PROBLEM = ROOT / "examples/showcases/x15_rocket_to_hawaii/short_range_propnav_6dof.prb"
X15_SURFACE_HOLD_TABLES = tuple(
    ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1/tables" / name
    for name in (
        "x15_static_6axis.tbl",
        "x15_symmetric_stabilator_6axis.tbl",
        "x15_differential_stabilator_6axis.tbl",
        "x15_rudder_6axis.tbl",
    )
)
GLIDER_TRIM_TABLES = X15_SURFACE_HOLD_TABLES

pytestmark = [pytest.mark.slow, pytest.mark.x15]


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


def _run_glider_6dof(problem: Path, output_dir: Path, *, max_steps: int = 6_000) -> RunReport:
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
    # The source-research trajectory grazes the reference surface by a few
    # millimetres in the final numerical samples; keep that explicit margin
    # separate from a future contact/ground-effect model.
    require_bounded(history, "altitude_m", minimum=-1.0, maximum=30_000.0)
    require_bounded(history, "mach", minimum=1.0, maximum=6.7)
    require_bounded(history, "aero_alpha_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "mass_kg", minimum=14641.0545, maximum=14641.0545)
    require_bounded(history, "thrust_n", minimum=0.0, maximum=0.0)
    require_bounded(history, "propellant_mass_kg", minimum=0.0, maximum=0.0)
####


def test_glider_6dof_source_trim_hold_stays_inside_strict_envelope(tmp_path: Path) -> None:
    """A source-table rudder trim establishes a bounded 30-second plant case."""

    report = run_files(
        GLIDER_TRIM_HOLD_6DOF,
        GLIDER_TRIM_TABLES,
        output_dir=tmp_path,
        max_steps=1_600,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert history[-1]["time_s"] == pytest.approx(30.0, abs=1.0e-10)
    require_bounded(history, "altitude_m", minimum=0.0, maximum=24_384.0)
    require_bounded(history, "mach", minimum=0.3, maximum=6.7)
    require_bounded(history, "aero_alpha_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
    assert abs(history[-1]["aero_sideslip_deg"]) < 0.1
    assert max(sample["rudder-deg"] for sample in history) >= 1.0
    assert min(sample["rudder-deg"] for sample in history) <= 2.0
####


def test_glider_6dof_native_propnav_activates_after_source_trim(tmp_path: Path) -> None:
    """Native ProNav activates through a normal segment transition after trim."""

    report = run_files(
        GLIDER_TRIM_THEN_PROPNAV_6DOF,
        GLIDER_TRIM_TABLES,
        output_dir=tmp_path,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    trim = [sample for sample in history if int(sample["_segment"]) == 1]
    propnav = [sample for sample in history if int(sample["_segment"]) == 2]
    assert history[-1]["time_s"] == pytest.approx(35.0, abs=1.0e-10)
    assert trim and propnav
    assert all(sample["pro_nav_active"] == pytest.approx(1.0) for sample in propnav)
    assert propnav[-1]["range_to_target_m"] < propnav[0]["range_to_target_m"]
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
    assert abs(propnav[-1]["aero_sideslip_deg"]) < 2.0
####


def test_x15_direct_untrimmed_propnav_fails_closed_at_beta_envelope(tmp_path: Path) -> None:
    """The unsafe direct launch path is rejected before false capture evidence."""

    report = run_files(
        X15_SURFACE_HOLD_PROBLEM,
        X15_SURFACE_HOLD_TABLES,
        output_dir=tmp_path,
        max_steps=2_000,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert report.exit_code != 0
    messages = tuple(item.message for item in report.diagnostics)
    assert any("outside its declared envelope" in message for message in messages), messages
    assert not any("non-finite" in message.casefold() for message in messages), messages
    ####


def test_glider_6dof_energy_contract_has_apogee_and_post_apogee_loss(tmp_path: Path) -> None:
    """The unpowered run has an identifiable apogee and loses specific energy."""

    report = _run_glider_6dof(GLIDER_6DOF, tmp_path)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    energies = specific_energy(history)
    altitudes = tuple(sample["altitude_m"] for sample in history)
    apogee_index = max(range(len(altitudes)), key=altitudes.__getitem__)
    assert 0 < apogee_index < len(history) - 1
    assert altitudes[apogee_index] > altitudes[0]
    assert altitudes[-1] < altitudes[apogee_index]
    assert energies[-1] < energies[0]
    assert all(sample["thrust_n"] == pytest.approx(0.0) for sample in history)
    assert all(sample["propellant_mass_rate_kg_s"] == pytest.approx(0.0) for sample in history)
    force_closure = independent_force_closure(history)
    assert force_closure["p99_normalized_residual"] < 1.0e-3
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 4948.7355,
            "inertia_y_kg_m2": 129114.2666,
            "inertia_z_kg_m2": 131825.9024,
        }
        for sample in history
    )
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 2.0e-2
####


def test_glider_bank_reversal_contract_is_explicitly_machine_checked(tmp_path: Path) -> None:
    """Native bank control recovers without leaving the source envelope."""

    report = _run_glider_6dof(GLIDER_BANK_REVERSAL_6DOF, tmp_path, max_steps=12_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    positive = [sample for sample in history if 0.0 <= sample["time_s"] < 20.0]
    negative = [sample for sample in history if 20.0 <= sample["time_s"] < 40.0]
    assert max(sample["bank_command_deg"] for sample in positive) >= 20.0
    assert min(sample["bank_command_deg"] for sample in negative) <= -20.0
    assert all(math.isfinite(sample["bank_achieved_deg"]) for sample in history)
    # The research trajectory grazes the reference surface by a few
    # millimetres in its final numerical samples; contact and ground effect
    # remain outside this source-surrogate claim.
    require_bounded(history, "altitude_m", minimum=-1.0, maximum=30_000.0)
    require_bounded(history, "mach", minimum=1.0, maximum=6.7)
    require_bounded(history, "aero_alpha_deg", minimum=-10.0, maximum=10.0)
    require_bounded(history, "aero_sideslip_deg", minimum=-10.0, maximum=10.0)
    # The commanded bank reversals are discrete control events; use the
    # documented research tolerance for a full history containing those
    # transitions rather than the smoother open-loop glide threshold.
    assert independent_force_closure(history)["p99_normalized_residual"] < 1.0e-2
    enriched = tuple(
        {
            **sample,
            "inertia_x_kg_m2": 4948.7355,
            "inertia_y_kg_m2": 129114.2666,
            "inertia_z_kg_m2": 131825.9024,
        }
        for sample in history
    )
    # The source surrogate's abrupt bank commands and sparse published moment
    # derivatives produce a larger, explicitly disclosed rotational residual
    # than the smooth glide case.
    assert independent_moment_closure(enriched)["p99_normalized_residual"] < 1.0e-1
####


def test_glider_great_circle_route_geometry_is_source_anchored(tmp_path: Path) -> None:
    """Route telemetry starts at the actual source release point and stays diagnostic."""

    report = _run_glider_6dof(GLIDER_ROUTE_GEOMETRY_6DOF, tmp_path, max_steps=6_000)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    history = _history(report)
    assert all(math.isfinite(sample["route_cross_track_error_m"]) for sample in history)
    assert all(-180.0 <= sample["route_heading_error_deg"] <= 180.0 for sample in history)
    assert abs(history[0]["route_cross_track_error_m"]) < 1.0
    assert max(abs(sample["route_cross_track_error_m"]) for sample in history) < 200_000.0
    # This is geometry evidence, not a terminal-capture claim.  The current
    # unpowered case remains intentionally separate from ProNav tuning.
    assert history[-1]["range_to_target_m"] > 1_000_000.0


def test_x15_surface_holds_use_declared_stabilator_controls(tmp_path: Path) -> None:
    """Generic alpha/beta holds bind X-15 stabilators through normal ingestion."""

    problem = tmp_path / "x15-surface-holds.prb"
    source = X15_SURFACE_HOLD_PROBLEM.read_text(encoding="utf-8")
    source = source.replace(
        "surface-pitch-gain-deg-per-rad=12.0",
        "surface-pitch-gain-deg-per-rad=0.0 alpha-hold-gain-deg-per-deg=0.5 "
        "alpha-hold-target-deg=3.1 differential-stabilator-hold-gain-deg-per-deg=1.0",
    )
    problem.write_text(source, encoding="utf-8")
    report = run_files(problem, X15_SURFACE_HOLD_TABLES, output_dir=tmp_path / "run", max_steps=2, profile=GrammarProfile.TAORYX)
    assert report.results
    history = report.results[0].states["1"]
    assert len(history) >= 2
    assert history[1].named["aero_query_symmetric-stabilator-deg"] != pytest.approx(0.0)
    assert history[1].named["aero_query_differential-stabilator-deg"] != pytest.approx(0.0)
    assert "aero_query_elevator-deg" not in history[1].named
    assert "aero_query_differential-elevon-deg" not in history[1].named
####


def test_x15_initial_airflow_release_alignment_is_applied_before_first_rhs(tmp_path: Path) -> None:
    """An initial release directive is active before table evaluation begins."""

    problem = tmp_path / "x15-initial-airflow-release.prb"
    source = X15_SURFACE_HOLD_PROBLEM.read_text(encoding="utf-8")
    # Permit inspection of the first RHS without turning this seam test into
    # a ProNav-robustness claim; the native X-15 beta envelope remains the
    # separately tracked controller diagnostic.
    source = source.replace("envelope-max-beta-deg=10", "envelope-max-beta-deg=90 release-attitude=airflow")
    problem.write_text(source, encoding="utf-8")
    report = run_files(problem, X15_SURFACE_HOLD_TABLES, output_dir=tmp_path / "run", max_steps=1, profile=GrammarProfile.TAORYX)
    assert report.results
    initial = report.results[0].states["1"][0].named
    assert initial["release_attitude_aligned"] == pytest.approx(1.0)
    assert initial["aero_sideslip_deg"] == pytest.approx(0.0, abs=1.0e-8)
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
        report = _run_glider_6dof(problem, output_dir / f"dt-{factor:g}", max_steps=math.ceil(6_000 / factor))
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
        channels=(
            "taos.altitude_m",
            "taos.speed_m_s",
            "taos.mach",
            "taos.aero_alpha_deg",
            "taos.aero_sideslip_deg",
            "taos.range_to_target_m",
            "taos.route_cross_track_error_m",
            "taos.route_heading_error_deg",
        ),
        )
    assert (destination / "convergence-report.json").stat().st_size > 100
    assert tuple((destination / "plots").glob("*.png"))
####
