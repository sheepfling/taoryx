from __future__ import annotations

import json
from pathlib import Path

import pytest

from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.cli import main
from taoryx.runtime.engine import run_taos
from taoryx.runtime.lowering import lower_problem_document, lower_tables
from taoryx.runtime.runner import run_files

ROOT = Path(__file__).resolve().parents[2]
PROBLEM = ROOT / "examples/chapter04/ballistic-reentry.prb"
TABLE = ROOT / "examples/chapter04/ballistic-reentry.tbl"


def test_lowering_expands_surveys_and_prepares_tables() -> None:
    problem = parse_problem_file(PROBLEM)
    table = parse_table_file(TABLE)
    tables = lower_tables(table)
    lowered = lower_problem_document(problem, tables)
    assert len(lowered.cases) == 16
    assert lowered.tables["ca-ex-1"].evaluate({"mach": 8.0}) == 0.0742
    assert lowered.cases[0].problem.vehicles["1"].state.named["vel"] == 15000.0
####


def test_file_runner_emits_case_outputs_and_report(tmp_path: Path) -> None:
    report = run_files(PROBLEM, (TABLE,), output_dir=tmp_path)
    assert report.exit_code == 0
    assert report.cases == 16
    assert (tmp_path / "1.print").exists()
    assert (tmp_path / "ex1.dbf").exists()
    summary = next(tmp_path.glob("case-1-summaries.json"))
    assert "max-q" in json.loads(summary.read_text(encoding="utf-8"))
####


def test_run_taos_accepts_problem_and_table_files(tmp_path: Path) -> None:
    report = run_taos(PROBLEM, (TABLE,), output_dir=tmp_path)

    assert report.exit_code == 0
    assert report.cases == 16
####


def test_full_table_define_fixture_writes_trajectory_file(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p010_output_define_full_table/input/p010_output_define_full_table.prb"
    table = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p010_output_define_full_table/input/output.tbl"
    report = run_files(problem, (table,), output_dir=tmp_path)
    assert report.exit_code == 0
    assert (tmp_path / "heat.dat").exists()
    assert "heat-proxy-value" in (tmp_path / "heat.dat").read_text(encoding="utf-8")
####


def test_propulsion_fixture_reaches_time_event(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p002_constant_thrust_table/input/p002_constant_thrust_table.prb"
    tables = tuple((problem.parent / name) for name in ("constant-thrust.tbl", "zero-mdot.tbl"))
    report = run_files(problem, tables, output_dir=tmp_path, max_steps=500)
    assert report.exit_code == 0
    assert report.results[0].completed
    assert report.results[0].states["1"][-1].time == pytest.approx(5.0)
####


def test_search_fixture_recomputes_trajectory_until_target(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p021_search_linear_target/input/p021_search_linear_target.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(100.0, abs=1e-3)
    assert final.named["x"] == pytest.approx(20926646.3255, abs=1e-3)
####


def test_optimize_fixture_recomputes_scalar_boundary(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p022_optimize_linear_boundary/input/p022_optimize_linear_boundary.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["xdt"] == pytest.approx(100.0, abs=1e-2)
    assert final.named["x"] == pytest.approx(20926646.3255, abs=0.1)
####


def test_user_atmosphere_refreshes_environment_outputs(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p012_user_atmosphere/input/p012_user_atmosphere.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=100)

    assert report.exit_code == 0
    lines = (tmp_path / "atmosphere.dat").read_text(encoding="utf-8").splitlines()
    assert all("483.03" in line and "1455.3" in line and "0.001756" in line for line in lines[1:])
####


def test_aero_table_uses_environment_pressure_and_reference_area(tmp_path: Path) -> None:
    problem = tmp_path / "aero.prb"
    problem.write_text(
        "(aero)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 300 100 1 100 1\n"
        "100 300 100 1 100 1\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=100 gama=0 psi=0 time=0 mass=1\n"
        "  *file aero.dat time vel dynprs mach ca\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *aero ca=(drag)\n"
        "    *when time=0.2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = tmp_path / "drag.tbl"
    table.write_text("(drag)\ntable ca(mach) sref=2\nmach=0,2\nca=0.01,0.01\n", encoding="utf-8")

    report = run_files(problem, (table,), output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    lines = (tmp_path / "out" / "aero.dat").read_text(encoding="utf-8").splitlines()
    assert "5000.0" in lines[1]
    assert "1.0" in lines[1]
    assert float(lines[-1].split()[1]) < 100.0
    assert "0.01" in lines[-1]
####


def test_summary_selects_trajectory_history_and_alias(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p020_survey_summarize/input/p020_survey_summarize.prb"
    report = run_files(problem, output_dir=tmp_path, max_steps=500)

    assert report.exit_code == 0
    summary = json.loads((tmp_path / "case-1-summaries.json").read_text(encoding="utf-8"))
    assert summary["final-x"] == pytest.approx(20925746.3255, abs=1e-3)
####


def test_direct_fly_control_refreshes_runtime_state(tmp_path: Path) -> None:
    problem = tmp_path / "fly.prb"
    problem.write_text(
        "(fly)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 aircraft start on 1\n"
        "  *initial geodetic alt=0 long=0 lat=0 vel=10 gama=0 psi=0 time=0 mass=1\n"
        "  *file fly.dat time mach\n"
        "  *segment 1 cruise\n"
        "    *integ dt=0.1\n"
        "    *fly mach=0.55\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    assert "0.55" in (tmp_path / "out" / "fly.dat").read_text(encoding="utf-8")
####


def test_multi_parameter_optimization_recomputes_constraint(tmp_path: Path) -> None:
    problem = tmp_path / "optimize-two.prb"
    problem.write_text(
        "(optimize-two)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=opta-1 ydt=opta-2 zdt=0 time=0 mass=1\n"
        "  *file optimize-two.dat time xecfc yecfc xecfcdt yecfcdt\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 stop\n"
        "*optimize a for xecfc=max on segment 1, trajectory 1\n"
        "  constrain yecfc=10 on segment 1, trajectory 1\n"
        "  par-1=5 lo-1=0 hi-1=20 par-2=0 lo-2=-20 hi-2=20 maxitr=30 tol=0.001\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=200)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["x"] == pytest.approx(20.0, abs=0.2)
    assert final.named["y"] == pytest.approx(10.0, abs=0.2)
####


def test_ecfc_fixture_integrates_cartesian_state_and_alias_outputs(tmp_path: Path) -> None:
    problem = ROOT / "tests/fixtures/taos_e2e_v23/cases/positive/p001_linear_ecfc_zero_force/input/p001_linear_ecfc_zero_force.prb"
    report = run_files(problem, output_dir=tmp_path)

    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.named["x"] == pytest.approx(20925746.3255)
    assert final.named["y"] == pytest.approx(80.0)
    assert "10.0 20925746.3255 80.0 -40.0 10.0 -2.0 1.0 10.0" in (tmp_path / "linear.dat").read_text(encoding="utf-8")
####


def test_goto_applies_segment_reset_and_increment(tmp_path: Path) -> None:
    problem = tmp_path / "segments.prb"
    problem.write_text(
        "(segments)\n"
        "*trajectory 1 rocket start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 wt=1\n"
        "  *segment 1 first\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 second\n"
        "    *reset wt=2\n"
        "    *increment vel=3\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)
    assert report.exit_code == 0
    final = report.results[0].states["1"][-1]
    assert final.time == 2.0
    assert final.named["wt"] == 2.0
    assert final.named["vel"] == 3.0
####


def test_inherited_trajectory_activates_at_source_segment(tmp_path: Path) -> None:
    problem = tmp_path / "branch.prb"
    problem.write_text(
        "(branch)\n"
        "*trajectory 1 parent start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>1 goto 2\n"
        "  *segment 2 handoff\n"
        "    *when tseg>0 stop\n"
        "*trajectory 2 child start on 1\n"
        "  *initial from trajectory 1, segment 2\n"
        "  *segment 1 child\n"
        "    *integ dt=0.1\n"
        "    *when time>2 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    report = run_files(problem, output_dir=tmp_path / "out", max_steps=100)

    assert report.exit_code == 0
    child = report.results[0].states["2"]
    assert child[0].time == pytest.approx(1.0)
    assert child[0].named["x"] == pytest.approx(1.0)
    assert child[-1].time == pytest.approx(2.0)
####


def test_run_cli_returns_input_error_and_writes_json_report(tmp_path: Path, capsys) -> None:
    report = tmp_path / "run.json"
    exit_code = main(["run", str(tmp_path / "missing.prb"), "--report", str(report), "--json"])
    assert exit_code == 2
    assert json.loads(report.read_text(encoding="utf-8"))["diagnostics"][0]["code"] == "problem-ingest-failed"
    assert "problem-ingest-failed" in capsys.readouterr().out
####


def test_lowering_marks_optimization_as_unsupported() -> None:
    document = parse_problem_text(
        "(optimization)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial ecfc x=0 y=0 z=0 xdt=0 ydt=0 zdt=0 mass=1\n"
        "*segment 1 coast\n"
        "*when time=1 stop\n"
        "*optimize xecfc=max on segment 1, trajectory 1\n"
        "*end\n"
    )

    lowered = lower_problem_document(document)

    assert lowered.unsupported_features == ("optimize",)
####
