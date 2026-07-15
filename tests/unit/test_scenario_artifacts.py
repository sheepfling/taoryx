from __future__ import annotations

import csv
import sqlite3
from pathlib import Path

from taoryx.scenario import InitialValueOverride, ScenarioCompiler

ROOT = Path(__file__).resolve().parents[2]


def test_scenario_run_produces_identity_linked_artifact_and_metadata(tmp_path: Path) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
        patches=(InitialValueOverride(vehicle="1", field="mass", value=500.0),),
    )

    artifact = scenario.run(output_dir=tmp_path, max_steps=20_000)[0]
    assert artifact.scenario_identity == scenario.identity
    assert artifact.visualization["source"] == "RunArtifact"
    assert artifact.resolution_records[0]["target"] == "1.mass"

    database = artifact.write_sqlite(tmp_path / "run.sqlite")
    with sqlite3.connect(database) as connection:
        assert connection.execute(
            "SELECT value_json FROM taoryx_run_metadata WHERE name = 'scenario_identity'"
        ).fetchone() == (f'"{scenario.identity}"',)
    ####


def test_artifact_sinks_reconcile_sample_and_channel_counts(tmp_path: Path) -> None:
    scenario = ScenarioCompiler().compile(
        ROOT / "examples/chapter04/ballistic-reentry.prb",
        table_paths=(ROOT / "examples/chapter04/ballistic-reentry.tbl",),
    )
    artifact = scenario.run(output_dir=tmp_path / "run", max_steps=20_000)[0]
    vehicle = artifact.vehicles["1"]
    json_path = artifact.write_json(tmp_path / "run.json")
    restored = type(artifact).model_validate_json(json_path.read_text(encoding="utf-8"))
    csv_path = artifact.write_csv(tmp_path / "run.csv", vehicle_id="1")
    sqlite_path = artifact.write_sqlite(tmp_path / "run.sqlite")

    expected_rows = len(vehicle.times) * len(vehicle.channels)
    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
        assert len(rows) == expected_rows
        assert {row["scenario_identity"] for row in rows} == {scenario.identity}
    with sqlite3.connect(sqlite_path) as connection:
        assert connection.execute("SELECT COUNT(*) FROM taoryx_samples").fetchone() == (expected_rows,)
        assert connection.execute("SELECT COUNT(*) FROM taoryx_channels").fetchone() == (len(vehicle.channels),)
    assert restored.scenario_identity == scenario.identity
    assert set(restored.vehicles["1"].channels) == set(vehicle.channels)
    ####
