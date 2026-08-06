from __future__ import annotations

import json
from pathlib import Path

from taoryx.runtime.cli import main
from taoryx.runtime.runner import run_files


def _write_runtime_inputs(root: Path, *, output_name: str = "result.dat") -> tuple[Path, Path]:
    problem = root / "demo.prb"
    problem.write_text(
        "(demo)\n"
        "*atmos none\n"
        "*earth spherical gm=0 omega=0\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 time=0 mass=1\n"
        "  *define sampled\n"
        "    sampled = table(gain, vel);\n"
        f"  *file {output_name} time sampled xecfc\n"
        "  *segment 1 coast\n"
        "    *integ dt=0.1\n"
        "    *when time>0.1 stop\n"
        "*end\n",
        encoding="utf-8",
    )
    table = root / "gain.tbl"
    table.write_text(
        "(gain)\n"
        "table ca(vel)\n"
        "vel=0,100\n"
        "ca=2,4\n",
        encoding="utf-8",
    )
    return problem, table


def test_cli_run_ingests_problem_and_table_and_writes_report(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path)
    output_dir = tmp_path / "output"
    report_path = tmp_path / "reports" / "run.json"
    artifact_path = tmp_path / "reports" / "artifact.json"

    exit_code = main(
        [
            "run",
            str(problem),
            str(table),
            "--output-dir",
            str(output_dir),
            "--report",
            str(report_path),
            "--artifact",
            str(artifact_path),
            "--json",
            "--max-steps",
            "20",
        ]
    )

    assert exit_code == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["cases"] == 1
    assert report["exit_code"] == 0
    assert report["status"] == "passed"
    assert report["normalized_artifact"] == str(artifact_path)
    assert json.loads(artifact_path.read_text(encoding="utf-8"))["schema_version"] == 1
    assert (output_dir / "result.dat").exists()
    assert json.loads(capsys.readouterr().out)["outputs"]
####


def test_cli_lists_available_integrators(capsys) -> None:
    assert main(["integrators", "list"]) == 0
    output = capsys.readouterr().out
    assert "euler\tfixed-step, fast smoke tests" in output
    assert "rk4\tfixed-step, recommended for deterministic production runs" in output
    assert "rkf45\tadaptive reference integrator" in output
####


def test_cli_accepts_selected_integrator(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path)

    exit_code = main(
        [
            "run",
            str(problem),
            str(table),
            "--integrator",
            "rkf45",
            "--output-dir",
            str(tmp_path / "output"),
            "--max-steps",
            "20",
        ]
    )

    assert exit_code == 0
    assert "executed 1 case" in capsys.readouterr().out


def test_cli_accepts_random_seed_option(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path)

    exit_code = main(
        [
            "run",
            str(problem),
            str(table),
            "--seed",
            "7",
            "--output-dir",
            str(tmp_path / "output"),
            "--max-steps",
            "20",
        ]
    )

    assert exit_code == 0
    assert "executed 1 case" in capsys.readouterr().out
####


def test_cli_run_rejects_output_that_escapes_output_directory(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path, output_name="../outside.dat")
    output_dir = tmp_path / "output"

    exit_code = main(["run", str(problem), str(table), "--output-dir", str(output_dir)])

    assert exit_code == 2
    assert "unsafe-output-path" in capsys.readouterr().out
    assert not (tmp_path / "outside.dat").exists()


def test_cli_run_rejects_nonpositive_step_limit(tmp_path: Path) -> None:
    problem, _ = _write_runtime_inputs(tmp_path)

    try:
        main(["run", str(problem), "--max-steps", "0"])
    except SystemExit as error:
        assert error.code == 2
    else:
        raise AssertionError("argparse should reject a nonpositive step limit")


def test_cli_run_reports_incomplete_case_when_step_limit_is_reached(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path)
    problem.write_text(problem.read_text(encoding="utf-8").replace("time>0.1", "time>10.0"), encoding="utf-8")

    exit_code = main(["run", str(problem), str(table), "--output-dir", str(tmp_path / "output"), "--max-steps", "1"])

    assert exit_code == 1
    assert "runtime-incomplete" in capsys.readouterr().out


def test_cli_run_reports_missing_table_dependencies_before_execution(tmp_path: Path, capsys) -> None:
    problem = tmp_path / "missing-table.prb"
    problem.write_text(
        "(missing-table)\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 mass=1\n"
        "  *segment 1 coast\n"
        "    *aero ca=(not-supplied)\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    exit_code = main(["run", str(problem), "--output-dir", str(tmp_path / "output")])

    assert exit_code == 2
    assert "missing-runtime-table" in capsys.readouterr().out
    assert not (tmp_path / "output" / "1.print").exists()


def test_cli_run_reports_missing_table_used_by_definition(tmp_path: Path, capsys) -> None:
    problem = tmp_path / "missing-definition-table.prb"
    problem.write_text(
        "(missing-definition-table)\n"
        "*trajectory 1 vehicle start on 1\n"
        "  *initial ecfc x=0 y=0 z=0 xdt=1 ydt=0 zdt=0 mass=1\n"
        "  *define gain\n"
        "    gain = table(not-supplied, 1);\n"
        "  *segment 1 coast\n"
        "    *when time>1 stop\n"
        "*end\n",
        encoding="utf-8",
    )

    exit_code = main(["run", str(problem), "--output-dir", str(tmp_path / "output")])

    assert exit_code == 2
    assert "missing-runtime-table" in capsys.readouterr().out


def test_cli_run_reports_unavailable_declared_output(tmp_path: Path, capsys) -> None:
    problem, table = _write_runtime_inputs(tmp_path, output_name="result.dat")
    problem.write_text(problem.read_text(encoding="utf-8").replace("time sampled xecfc", "time not_a_runtime_variable"), encoding="utf-8")

    exit_code = main(["run", str(problem), str(table), "--output-dir", str(tmp_path / "output")])

    assert exit_code == 2
    output = capsys.readouterr().out
    assert "runtime-execution-failed" in output
    assert "not_a_runtime_variable" in output


def test_runtime_failure_preserves_lowered_case_count(tmp_path: Path) -> None:
    problem, table = _write_runtime_inputs(tmp_path, output_name="result.dat")
    problem.write_text(problem.read_text(encoding="utf-8").replace("time sampled xecfc", "time not_a_runtime_variable"), encoding="utf-8")

    report = run_files(problem, (table,), output_dir=tmp_path / "output")

    assert report.exit_code == 2
    assert report.cases == 1
    assert any(item.code == "runtime-execution-failed" for item in report.diagnostics)


def test_cli_run_rejects_ambiguous_unqualified_table_across_inputs(tmp_path: Path, capsys) -> None:
    problem, _ = _write_runtime_inputs(tmp_path)
    first = tmp_path / "first.tbl"
    second = tmp_path / "second.tbl"
    first.write_text("(gain)\ntable ca(vel)\nvel=0,1\nca=1,1\n", encoding="utf-8")
    second.write_text("(GAIN)\ntable ca(vel)\nvel=0,1\nca=2,2\n", encoding="utf-8")

    exit_code = main(
        [
            "run",
            str(problem),
            str(first),
            str(second),
            "--output-dir",
            str(tmp_path / "output"),
        ]
    )

    assert exit_code == 2
    output = capsys.readouterr().out
    assert "missing-runtime-table" in output
    assert "gain" in output
