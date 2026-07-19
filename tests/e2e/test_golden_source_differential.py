"""Source-grid versus runtime-table differential checks for golden plants.

These tests establish the narrowest useful source-fidelity claim available in
the repository: the checked-in TAORYX tables reproduce their source CSV grid.
They do not claim agreement with a historical executable or a flight-qualified
vehicle model.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import pytest

from taoryx.table_explorer import inspect_table_file
from taoryx.tables import interpolate_nd

ROOT = Path(__file__).resolve().parents[2]
BUNDLE = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1"

pytestmark = pytest.mark.dof6


def _source_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))
    ####


def _runtime_tables(path: Path) -> dict[str, object]:
    artifact = inspect_table_file(path)
    return {table.name.casefold(): table.prepared for table in artifact.tables if table.prepared is not None}
    ####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _grid_report(
    rows: list[dict[str, str]],
    tables: dict[str, object],
    query_builder: Any,
) -> dict[str, Any]:
    """Compare every source row and retain reviewable worst-case evidence."""

    channels = tuple(sorted(tables))
    maximum_abs = {name: 0.0 for name in channels}
    maximum_rel = {name: 0.0 for name in channels}
    worst_case: dict[str, Any] = {}
    for row in rows:
        query = query_builder(row)
        for name in channels:
            prepared = tables[name]
            assert prepared is not None
            expected = float(row[name.upper()])
            actual = interpolate_nd(prepared, query)
            absolute = abs(actual - expected)
            relative = absolute / max(abs(expected), 1.0e-30)
            if absolute > maximum_abs[name]:
                maximum_abs[name] = absolute
                worst_case[name] = {
                    "source_row": dict(row),
                    "expected": expected,
                    "actual": actual,
                    "absolute_error": absolute,
                    "relative_error": relative,
                }
            maximum_rel[name] = max(maximum_rel[name], relative)
    return {
        "sample_count": len(rows),
        "channels": list(channels),
        "maximum_absolute_error": maximum_abs,
        "maximum_relative_error": maximum_rel,
        "worst_case": worst_case,
    }
    ####


def _b747_rows() -> list[dict[str, str]]:
    return [
        row
        for row in _source_rows(BUNDLE / "jet_b747/aero/static_six_axis_grid.csv")
        if row["configuration"] == "nominal"
        and row["reference_fc_id"] == "3"
        and row["mach"] == "0.45"
        and all(float(row[name]) == 0.0 for name in ("elevator_deg", "aileron_deg", "rudder_deg", "p_hat", "q_hat", "r_hat", "alpha_dot_hat"))
    ]
    ####


def _x8_rows() -> list[dict[str, str]]:
    return [
        row
        for row in _source_rows(BUNDLE / "cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv")
        if all(float(row[name]) == 0.0 for name in ("collective_elevon_deg", "differential_elevon_deg", "throttle", "p_hat", "q_hat", "r_hat"))
    ]
    ####


def source_differential_reports() -> dict[str, dict[str, Any]]:
    """Return machine-readable transcode evidence for the static source grids."""

    b747_source = BUNDLE / "jet_b747/aero/static_six_axis_grid.csv"
    b747_table = BUNDLE / "tables/b747_nominal_static_6axis.tbl"
    x8_source = BUNDLE / "cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv"
    x8_table = BUNDLE / "tables/skywalker_x8_static_6axis.tbl"
    reports = {
        "b747-condition3-static": _grid_report(
            _b747_rows(),
            _runtime_tables(b747_table),
            lambda row: (math.radians(float(row["alpha_offset_from_reference_deg"])), math.radians(float(row["beta_deg"]))),
        ),
        "skywalker-x8-zero-control-static": _grid_report(
            _x8_rows(),
            _runtime_tables(x8_table),
            lambda row: (
                float(row["velocity_m_s"]),
                float(row["altitude_m"]),
                math.radians(float(row["alpha_deg"])),
                math.radians(float(row["beta_deg"])),
            ),
        ),
    }
    for key, source, table in (
        ("b747-condition3-static", b747_source, b747_table),
        ("skywalker-x8-zero-control-static", x8_source, x8_table),
    ):
        reports[key]["source_sha256"] = _sha256(source)
        reports[key]["runtime_table_sha256"] = _sha256(table)
        reports[key]["comparison_scope"] = "source CSV grid versus ingested TAORYX table"
    return reports
    ####


def test_b747_condition3_table_reproduces_source_grid() -> None:
    rows = [
        row
        for row in _source_rows(BUNDLE / "jet_b747/aero/static_six_axis_grid.csv")
        if row["configuration"] == "nominal"
        and row["reference_fc_id"] == "3"
        and row["mach"] == "0.45"
        and all(float(row[name]) == 0.0 for name in ("elevator_deg", "aileron_deg", "rudder_deg", "p_hat", "q_hat", "r_hat", "alpha_dot_hat"))
    ]
    tables = _runtime_tables(BUNDLE / "tables/b747_nominal_static_6axis.tbl")
    assert len(rows) == 25
    assert set(tables) == {"cx", "cy", "cz", "cmx", "cmy", "cmz"}
    for row in rows:
        query = (math.radians(float(row["alpha_offset_from_reference_deg"])), math.radians(float(row["beta_deg"])))
        for name in tables:
            prepared = tables[name]
            assert prepared is not None
            actual = interpolate_nd(prepared, query)
            assert actual == pytest.approx(float(row[name.upper()]), abs=2.0e-10)
    ####


def test_source_differential_report_has_complete_provenance() -> None:
    reports = source_differential_reports()
    assert reports["b747-condition3-static"]["sample_count"] == 25
    assert reports["skywalker-x8-zero-control-static"]["sample_count"] == 840
    for report in reports.values():
        assert len(report["source_sha256"]) == 64
        assert len(report["runtime_table_sha256"]) == 64
        assert report["channels"] == ["cmx", "cmy", "cmz", "cx", "cy", "cz"]
        assert all(value <= 2.0e-10 for value in report["maximum_absolute_error"].values())
        assert report["worst_case"]
    ####


@pytest.mark.artifact
def test_source_differential_report_writes_review_artifact(artifact_dir: Path) -> None:
    destination = artifact_dir / "source-differential" / "golden_static_grid_report.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(source_differential_reports(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    assert destination.stat().st_size > 1_000
    ####


def test_x8_zero_control_table_reproduces_source_grid() -> None:
    rows = [
        row
        for row in _source_rows(BUNDLE / "cruise_class_uav_skywalker_x8/aero/static_airframe_grid.csv")
        if all(float(row[name]) == 0.0 for name in ("collective_elevon_deg", "differential_elevon_deg", "throttle", "p_hat", "q_hat", "r_hat"))
    ]
    tables = _runtime_tables(BUNDLE / "tables/skywalker_x8_static_6axis.tbl")
    assert len(rows) == 840
    assert set(tables) == {"cx", "cy", "cz", "cmx", "cmy", "cmz"}
    for row in rows:
        query = (
            float(row["velocity_m_s"]),
            float(row["altitude_m"]),
            math.radians(float(row["alpha_deg"])),
            math.radians(float(row["beta_deg"])),
        )
        for name in tables:
            prepared = tables[name]
            assert prepared is not None
            actual = interpolate_nd(prepared, query)
            assert actual == pytest.approx(float(row[name.upper()]), abs=2.0e-10)
    ####
