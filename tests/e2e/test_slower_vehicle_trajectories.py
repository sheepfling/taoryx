from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files
from taoryx.visualization import render_run_artifact_plots
from tests.e2e.support.golden_plants import GoldenPlantCase, run_golden_plant

ROOT = Path(__file__).resolve().parents[2]
MATRIX = yaml.safe_load((ROOT / "examples/mission_families/slower_vehicle_trajectory_matrix.yaml").read_text(encoding="utf-8"))
FIXTURE_TABLES = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
HUMMINGBIRD_HOVER = ROOT / "examples/mission_families/slower_hummingbird/SV05_hover_validation_6dof.prb"
HUMMINGBIRD_DISTURBED = ROOT / "examples/mission_families/slower_hummingbird/SV05_disturbed_response_6dof.prb"
B747_ANCHOR = ROOT / "examples/mission_families/slower_b747/SV01_anchor_condition3_6dof.prb"
B747_TRIM = ROOT / "examples/mission_families/slower_b747/SV01_condition3_trim_6dof.prb"
X8_ANCHOR = ROOT / "examples/mission_families/slower_x8/SV03_published_trim_anchor_6dof.prb"
X8_TRIM = ROOT / "examples/mission_families/slower_x8/SV03_powered_trim_6dof.prb"
X8_LATERAL = ROOT / "examples/mission_families/slower_x8/SV03_lateral_rate_response_6dof.prb"
X8_COMBINED = ROOT / "examples/mission_families/slower_x8/SV03_combined_controller_recovery_6dof.prb"

pytestmark = [pytest.mark.dof3, pytest.mark.dof6]


@pytest.mark.dof6
@pytest.mark.parametrize(
    ("problem", "tables", "expected_alpha_deg", "expected_beta_deg"),
    (
        (B747_ANCHOR, (FIXTURE_TABLES / "b747_nominal_static_6axis.tbl",), 3.1, 0.0),
        (X8_ANCHOR, (FIXTURE_TABLES / "skywalker_x8_static_6axis.tbl",), 7.9, 0.0),
    ),
    ids=("b747-condition3-alpha-reference", "x8-published-trim"),
)
def test_slower_vehicle_anchor_queries_are_in_envelope(
    problem: Path,
    tables: tuple[Path, ...],
    expected_alpha_deg: float,
    expected_beta_deg: float,
    tmp_path: Path,
) -> None:
    """The authoritative anchor states reach their tables before trajectory claims."""

    run = run_golden_plant(
        GoldenPlantCase(
            vehicle=problem.stem,
            problem=problem,
            tables=tables,
            max_steps=100,
        ),
        tmp_path,
    )
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_values(
        aero_alpha_deg=expected_alpha_deg,
        aero_sideslip_deg=expected_beta_deg,
    )
    run.require_initial_closure(translation_max=1.0e-10, rotation_max=1.0e-10)
    run.require_table_margins()
    ####


@pytest.mark.slow
@pytest.mark.dof6
def test_b747_condition3_trim_hold_is_bounded(tmp_path: Path) -> None:
    """The solved B747 condition-3 plant remains near its source anchor."""

    run = run_golden_plant(
        GoldenPlantCase(
            vehicle="B747-condition3-trim",
            problem=B747_TRIM,
            tables=(FIXTURE_TABLES / "b747_nominal_elevator_6axis.tbl",),
            max_steps=600,
        ),
        tmp_path,
    )
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_closure()
    run.require_table_margins()
    run.require_bounded_delta("altitude_m", 1.0)
    run.require_bounded_delta("speed_m_s", 1.0)
    run.require_channels("time_s", "aero_alpha_deg", "aero_sideslip_deg")
    assert run.final["time_s"] == pytest.approx(10.0, abs=1.0e-10)
    assert max(abs(sample["aero_alpha_deg"]) for sample in run.history) < 4.0
    assert max(abs(sample["aero_sideslip_deg"]) for sample in run.history) < 5.0
    ####


@pytest.mark.dof6
def test_x8_solved_powered_trim_closes_at_initial_state(tmp_path: Path) -> None:
    """The solved X8 trim is wired through all three coefficient families."""

    run = run_files(
        X8_TRIM,
        (
            FIXTURE_TABLES / "skywalker_x8_static_6axis.tbl",
            FIXTURE_TABLES / "skywalker_x8_collective_elevon_6axis.tbl",
            FIXTURE_TABLES / "skywalker_x8_differential_elevon_6axis.tbl",
            FIXTURE_TABLES / "skywalker_x8_thrust.tbl",
        ),
        output_dir=tmp_path,
        max_steps=2,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    assert run.exit_code == 0, [(item.code, item.message) for item in run.diagnostics]
    assert run.results
    state = run.results[0].states["1"][0].named
    assert state["translation_equation_residual_normalized"] < 1.0e-10
    assert state["rotation_equation_residual_normalized"] < 1.0e-10
    assert state["aero_alpha_deg"] == pytest.approx(7.7591088148, abs=1.0e-8)
    assert state["aero_sideslip_deg"] == pytest.approx(0.0, abs=1.0e-8)
    ####


@pytest.mark.slow
@pytest.mark.dof6
def test_x8_composed_differential_elevon_response_is_bounded(tmp_path: Path) -> None:
    """The composed control families produce a bounded five-second lateral response."""

    tables = tuple(
        FIXTURE_TABLES / name
        for name in (
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        )
    )
    run = run_files(X8_LATERAL, tables, output_dir=tmp_path / "active", max_steps=2000, integrator="rk4", profile=GrammarProfile.TAORYX)
    assert run.exit_code == 0, [(item.code, item.message) for item in run.diagnostics]
    assert run.results and run.results[0].completed
    history = run.results[0].states["1"]
    assert history[-1].time == pytest.approx(5.0, abs=1.0e-10)
    assert max(abs(state.named["aero_alpha_deg"]) for state in history) < 12.0
    assert max(abs(state.named["aero_sideslip_deg"]) for state in history) < 5.0
    assert max(state.named["aero_table_margin_table.cx-differential.beta"] for state in history) >= 0.0
    assert max(abs(state.named["aero_moment_body_z_nm"]) for state in history) > 1.0e-4

    zero_differential = tmp_path / "zero-differential.prb"
    zero_differential.write_text(X8_LATERAL.read_text(encoding="utf-8").replace("default=-2.16", "default=0.0"), encoding="utf-8")
    zero_run = run_files(zero_differential, tables, output_dir=tmp_path / "zero", max_steps=2000, integrator="rk4", profile=GrammarProfile.TAORYX)
    assert zero_run.exit_code == 0, [(item.code, item.message) for item in zero_run.diagnostics]
    active_initial = history[0].named["aero_moment_body_z_nm"]
    zero_initial = zero_run.results[0].states["1"][0].named["aero_moment_body_z_nm"]
    assert abs(active_initial - zero_initial) > 1.0e-5
    ####


@pytest.mark.slow
@pytest.mark.dof6
def test_x8_combined_longitudinal_lateral_recovery_is_bounded(tmp_path: Path) -> None:
    """The longitudinal and lateral holds share one composed coefficient deck."""

    tables = tuple(
        FIXTURE_TABLES / name
        for name in (
            "skywalker_x8_static_6axis.tbl",
            "skywalker_x8_collective_elevon_6axis.tbl",
            "skywalker_x8_differential_elevon_6axis.tbl",
            "skywalker_x8_thrust.tbl",
        )
    )
    run = run_files(X8_COMBINED, tables, output_dir=tmp_path, max_steps=2000, integrator="rk4", profile=GrammarProfile.TAORYX)
    assert run.exit_code == 0, [(item.code, item.message) for item in run.diagnostics]
    assert run.results and run.results[0].completed
    history = run.results[0].states["1"]
    assert history[-1].time == pytest.approx(5.0, abs=1.0e-10)
    assert min(state.named["speed_m_s"] for state in history) > 2.0
    assert max(abs(state.named["aero_alpha_deg"]) for state in history) < 12.0
    assert max(abs(state.named["aero_sideslip_deg"]) for state in history) < 5.0
    assert max(abs(state.named["aero_moment_body_y_nm"]) for state in history) > 1.0e-4
    assert max(abs(state.named["aero_moment_body_z_nm"]) for state in history) > 1.0e-4
    assert max(state.named["translation_equation_residual_normalized"] for state in history) < 1.0e-10
    assert max(state.named["rotation_equation_residual_normalized"] for state in history) < 1.0e-10
    for family in ("static", "collective", "differential"):
        key = f"aero_table_margin_table.cx-{family}.alpha"
        assert all(state.named[key] >= 0.0 for state in history)
    ####


def _run_pair(case: dict[str, object], output_dir: Path) -> tuple[RunReport, RunReport]:
    directory = ROOT / "examples/mission_families" / str(case["directory"])
    problem_3dof = directory / str(case["problem_3dof"])
    problem_6dof = directory / str(case["problem_6dof"])
    tables = tuple(FIXTURE_TABLES / str(path) for path in case["tables_6dof"])
    baseline = run_files(problem_3dof, tables, output_dir=output_dir / "3dof", max_steps=int(case["max_steps"]), profile=GrammarProfile.TAORYX)
    high_fidelity = run_files(
        problem_6dof,
        tables,
        output_dir=output_dir / "6dof",
        max_steps=int(case["max_steps"]),
        integrator=str(case["integrator"]),
        profile=GrammarProfile.TAORYX,
    )
    return baseline, high_fidelity
    ####


@pytest.mark.parametrize("case", MATRIX["cases"], ids=lambda case: str(case["id"]))
def test_slower_vehicle_3dof_then_6dof_pair(case: dict[str, object], tmp_path: Path) -> None:
    baseline, high_fidelity = _run_pair(case, tmp_path / str(case["id"]))
    assert baseline.exit_code == 0, [(item.code, item.message) for item in baseline.diagnostics]
    if high_fidelity.exit_code != 0:
        messages = [item.message for item in high_fidelity.diagnostics]
        assert any("outside its declared envelope" in message for message in messages), messages
        pytest.skip(f"{case['id']} smoke pair correctly rejected outside its verified coefficient envelope")
    assert baseline.results
    baseline_history = baseline.results[0].states["1"]
    high_history = high_fidelity.results[0].states["1"]
    assert len(baseline_history) >= 2
    assert len(high_history) >= 2
    # The 3-DOF leg is a reduction of the same verified coefficient and
    # propulsion tables, not the former cl=cd=0 placeholder.  Keep this
    # assertion close to the paired-run gate so a fixture cannot silently
    # regress to a ballistic plumbing test.
    baseline_initial = baseline_history[0].named
    if str(case["vehicle"]) != "hummingbird":
        assert all(name in baseline_initial for name in ("cx", "cy", "cz", "thrust"))
        assert all(baseline_initial[name] == baseline_initial[name] for name in ("cx", "cy", "cz", "thrust"))
    else:
        assert baseline_initial["thrust"] == pytest.approx(4.903, abs=0.02)
    tolerances = case["tolerances"]
    assert high_history[-1].time == pytest.approx(baseline_history[-1].time, abs=float(tolerances["time_s"]))
    assert all(value == value and abs(value) != float("inf") for state in high_history for value in state.values)
    final = high_history[-1].named
    assert final["aero_active"] == pytest.approx(1.0)
    quaternion_norm = sum(final[name] ** 2 for name in ("qw", "qx", "qy", "qz"))
    assert quaternion_norm == pytest.approx(1.0, abs=float(tolerances["quaternion_norm"]))
    if case["invariant"] == "direct_wrench_aero_active":
        assert max(abs(state.named["aero_force_body_z_n"]) for state in high_history) > 0.0
    else:
        assert max(abs(state.named["aero_moment_body_y_nm"]) for state in high_history) > 0.0
    ####


@pytest.mark.artifact
@pytest.mark.parametrize("case", MATRIX["cases"], ids=lambda case: str(case["id"]))
def test_slower_vehicle_pair_writes_comparison_artifacts(case: dict[str, object], artifact_dir: Path, tmp_path: Path) -> None:
    baseline, high_fidelity = _run_pair(case, tmp_path / str(case["id"]))
    assert baseline.results
    destination = artifact_dir / "slower-vehicle-pairs" / str(case["id"])
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "comparison.json").write_text(
        json.dumps({"case": case, "baseline": baseline.as_dict(), "six_dof": high_fidelity.as_dict()}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if not high_fidelity.results:
        messages = [item.message for item in high_fidelity.diagnostics]
        assert any("outside its declared envelope" in message for message in messages), messages
        (destination / "status.txt").write_text("6-DOF rejected outside declared coefficient envelope\n", encoding="utf-8")
        assert (destination / "comparison.json").stat().st_size > 100
        return
    for name, report in (("3dof", baseline), ("6dof", high_fidelity)):
        for artifact in report.artifacts:
            render_run_artifact_plots(
                artifact,
                destination / name,
                vehicle_id="1",
                channels=(
                    "taos.altitude_m", "taos.speed_m_s", "taos.aero_airspeed_m_s",
                    "taos.aero_dynamic_pressure_pa", "taos.aero_alpha_deg", "taos.aero_sideslip_deg",
                    "taos.aero_force_body_z_n",
                ),
            )
    assert (destination / "comparison.json").stat().st_size > 100
    ####


@pytest.mark.slow
@pytest.mark.dof6
def test_hummingbird_hover_validation_is_stationary(tmp_path: Path) -> None:
    """The source-frame rotor hover is a physical validation case, not a smoke pair."""

    tables = tuple(FIXTURE_TABLES / name for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    ))
    run = run_golden_plant(
        GoldenPlantCase(vehicle="Hummingbird-hover", problem=HUMMINGBIRD_HOVER, tables=tables, max_steps=2500),
        tmp_path,
    )
    run.require_success()
    run.require_finite()
    run.require_active_aerodynamics()
    run.require_convention_firewall()
    run.require_initial_closure(translation_max=1.0e-10, rotation_max=1.0e-10)
    run.require_channels("altitude_m", "speed_m_s", "aero_force_body_z_n", "total_force_body_z_n")
    assert run.final["altitude_m"] == pytest.approx(run.initial["altitude_m"], abs=0.25)
    assert run.final["speed_m_s"] < 0.1
    assert max(abs(sample["aero_force_body_z_n"]) for sample in run.history) > 4.0
    assert max(abs(sample[name]) for sample in run.history for name in ("aero_moment_body_x_nm", "aero_moment_body_y_nm", "aero_moment_body_z_nm")) < 1.0e-8
    assert max(abs(sample["total_force_body_z_n"]) for sample in run.history) < 1.0e-2
    assert max(sample["translation_equation_residual_normalized"] for sample in run.history) < 1.0e-10
    assert max(sample["rotation_equation_residual_normalized"] for sample in run.history) < 1.0e-10
    ####


@pytest.mark.slow
@pytest.mark.dof6
def test_hummingbird_disturbed_response_remains_finite_and_closed(tmp_path: Path) -> None:
    """A finite disturbed response exercises the full table envelope for 5 s."""

    tables = tuple(FIXTURE_TABLES / name for name in (
        "hummingbird_cx.tbl",
        "hummingbird_cy.tbl",
        "hummingbird_cz.tbl",
        "hummingbird_cmx.tbl",
        "hummingbird_cmy.tbl",
        "hummingbird_cmz.tbl",
    ))
    report = run_files(HUMMINGBIRD_DISTURBED, tables, output_dir=tmp_path, max_steps=2000, integrator="rk4", profile=GrammarProfile.TAORYX)
    assert report.exit_code == 0, [(item.code, item.message) for item in report.diagnostics]
    assert report.results and report.results[0].completed
    history = report.results[0].states["1"]
    assert history[-1].time == pytest.approx(5.0, abs=1.0e-12)
    assert all(value == value and abs(value) != float("inf") for state in history for value in state.values)
    assert max(state.named["speed_m_s"] for state in history) >= 0.5
    assert max(state.named["translation_equation_residual_normalized"] for state in history) < 1.0e-10
    assert max(state.named["rotation_equation_residual_normalized"] for state in history) < 1.0e-10
    ####
