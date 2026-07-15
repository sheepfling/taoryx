from __future__ import annotations

import json
from pathlib import Path

from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables

ROOT = Path("tests/fixtures/x15_coherent_6dof_public_research_v1")


def test_x15_source_validation_report_passes() -> None:
    report = json.loads((ROOT / "checks/validation_report.json").read_text(encoding="utf-8"))
    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert report["checks"]["actual_grid_counts"]["static_grid_zero_controls.csv"] == 3780


def test_x15_generated_tables_are_complete_and_parser_valid() -> None:
    expected = {
        "x15_static_6axis.tbl": ("mach", "altitude_m", "alpha", "beta"),
        "x15_symmetric_stabilator_6axis.tbl": ("mach", "altitude_m", "alpha", "beta", "symmetric_stabilator"),
        "x15_differential_stabilator_6axis.tbl": ("mach", "altitude_m", "alpha", "beta", "differential_stabilator"),
        "x15_rudder_6axis.tbl": ("mach", "altitude_m", "alpha", "beta", "rudder"),
    }
    for filename, axes in expected.items():
        document = parse_table_file(ROOT / "tables" / filename)
        assert document.diagnostics == []
        tables = lower_tables(document)
        assert set(tables) == {"cx", "cy", "cz", "cmx", "cmy", "cmz"}
        assert {table.independent_variables for table in tables.values()} == {axes}
        assert all(table.prepared is not None for table in tables.values())
    ####
