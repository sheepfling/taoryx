from __future__ import annotations

import sqlite3
from pathlib import Path

import pytest

from taoryx.outputs import DynamicsKind, RunArtifact, VehicleKind, build_output_evaluation_plan, build_run_artifact
from taoryx.runtime.common import RuntimeProblem, RuntimeState, RuntimeVehicle
from taoryx.runtime.engine import ExecutionResult


def test_output_plan_deduplicates_sources_and_reports_unavailable_evaluators() -> None:
    result = build_output_evaluation_plan(
        ("alt", "vel"),
        final_conditions=("vel", "mach"),
        expressions=("range",),
        available_evaluators={"alt", "vel"},
    )

    assert result.required == ("alt", "vel", "mach", "range")
    assert result.unavailable == ("mach", "range")


def test_ballistic_reentry_summary_listing_preserves_source_numeric_oracle() -> None:
    lines = Path("examples/chapter04/ballistic-reentry-summary.txt").read_text(encoding="utf-8").splitlines()
    header_index = next(index for index, line in enumerate(lines) if line.strip().startswith("ventry"))
    assert lines[header_index].split() == ["ventry", "gentry", "max_g", "max_q"]

    rows = [line.split() for line in lines[header_index + 1 :] if line.strip()]
    numeric_rows = [
        row
        for row in rows
        if len(row) == 4 and all(part.replace(".", "", 1).replace("-", "", 1).isdigit() for part in row)
    ]

    assert len(numeric_rows) == 16
    assert tuple(map(float, numeric_rows[0])) == (15000.0, -35.0, 34.481, 129020.0)
    assert tuple(map(float, numeric_rows[-1])) == (18000.0, -20.0, 31.966, 130690.0)
####


def test_problem_output_example_preserves_source_numeric_oracle() -> None:
    columns = Path("examples/chapter04/problem-output-example.txt").read_text(encoding="utf-8").splitlines()
    assert columns[0].split() == ["Time[1]", "Alt[1]", "Alt[2]", "Alt[3]", "Vel[1]", "Vel[2]", "Vel[3]"]

    first = [float(part) for part in columns[1].split()]
    last = [float(part) for part in columns[-1].split()]
    assert first == [0.0, 100000.0, 80000.0, 90000.0, 9000.0, 7000.0, 8000.0]
    assert last == [5.0, 100000.0, 80000.0, 90000.0, 8928.24, 6879.78, 7910.86]
####


def test_run_artifact_preserves_semantic_channels_and_segment_events(tmp_path: Path) -> None:
    vehicle = RuntimeVehicle(
        "1",
        RuntimeState(0.0, (0.0,), named={"alt": 0.0, "segment": 1.0}),
        active=True,
    )
    first = vehicle.state
    second = RuntimeState(1.0, (1.0,), named={"alt": 100.0, "segment": 2.0})
    artifact = build_run_artifact(
        "demo.prb",
        RuntimeProblem({"1": vehicle}, metadata={"parameters": {"launch_angle": 35.0}}),
        ExecutionResult({"1": (first, second)}, completed=True),
        vehicle_kinds={"1": VehicleKind.ROCKET},
    )

    telemetry = artifact.vehicles["1"]
    assert telemetry.kind is VehicleKind.ROCKET
    assert telemetry.dynamics is DynamicsKind.POINT_MASS_3DOF
    assert telemetry.channels["position.altitude.geodetic"].values == [0.0, 100.0]
    assert telemetry.channels["phase.segment"].interpolation == "step"
    assert [span.number for span in telemetry.segments] == [1, 2]
    assert telemetry.events[0].segment_from == 1
    assert telemetry.events[0].segment_to == 2
    assert artifact.parameters == {"launch_angle": 35.0}

    output = artifact.write_json(tmp_path / "telemetry.json")
    restored = RunArtifact.model_validate_json(output.read_text(encoding="utf-8"))
    assert restored == artifact


def test_run_artifact_print_and_sqlite_sinks_preserve_plotter_inputs(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    vehicle = RuntimeVehicle(
        "1",
        RuntimeState(0.0, (0.0,), named={"alt": 0.0, "segment": 1.0}),
        active=True,
    )
    artifact = build_run_artifact(
        "demo.prb",
        RuntimeProblem({"1": vehicle}, metadata={"parameters": {"launch_angle": 35.0}}),
        ExecutionResult(
            {"1": (vehicle.state, RuntimeState(1.0, (1.0,), named={"alt": 100.0, "segment": 1.0}))},
            completed=True,
        ),
    )

    text = artifact.format_text(max_rows=1)
    assert "Run: demo.prb" in text
    assert "position.altitude.geodetic" in text
    assert "... 1 rows omitted" in text
    artifact.print_text(max_rows=0)
    assert "Vehicle 1" in capsys.readouterr().out

    database = artifact.write_sqlite(tmp_path / "run.sqlite", run_id="case-1")
    with sqlite3.connect(database) as connection:
        assert connection.execute("SELECT problem FROM taoryx_runs WHERE run_id = 'case-1'").fetchone() == ("demo.prb",)
        assert connection.execute("SELECT COUNT(*) FROM taoryx_channels").fetchone() == (2,)
        assert connection.execute("SELECT COUNT(*) FROM taoryx_samples").fetchone() == (4,)
        assert connection.execute(
            "SELECT interpolation FROM taoryx_channels WHERE semantic_name = 'phase.segment'"
        ).fetchone() == ("step",)
        assert connection.execute("SELECT value FROM taoryx_samples WHERE semantic_name = 'position.altitude.geodetic' AND sample_index = 1").fetchone() == (100.0,)

    csv_path = artifact.write_csv(tmp_path / "run.csv", vehicle_id="1", channels=("position.altitude.geodetic",))
    rows = csv_path.read_text(encoding="utf-8").splitlines()
    assert rows[0] == "vehicle_id,time,semantic_name,source_name,unit,value"
    assert rows[-1].endswith(",100.0")
