from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import pytest

from taoryx.language.table_parser import parse_table_file
from taoryx.runtime.lowering import lower_tables

ROOT = Path("tests/fixtures/public_hypersonic_research_v1")


def test_public_hypersonic_source_validation_report_passes() -> None:
    report = json.loads((ROOT / "source/checks/validation_report.json").read_text(encoding="utf-8"))
    assert report["status"] == "PASS"
    assert report["failures"] == []
    assert report["checks"]["winged_cone_static_rows"] == 88


def test_langley_force_tables_are_parser_valid_and_preserve_source_rows() -> None:
    source_rows = list(csv.DictReader((ROOT / "source/aero/langley_winged_cone_hypersonic_static.csv").open(encoding="utf-8", newline="")))
    document = parse_table_file(ROOT / "tables/langley_winged_cone_6axis_static.tbl")
    assert document.diagnostics == []
    tables = lower_tables(document)
    assert set(tables) == {"cx", "cy", "cz", "cmx", "cmy", "cmz"}
    assert all(table.independent_variables == ("mach", "alpha") for table in tables.values())

    for row in source_rows:
        values = {"mach": float(row["mach"]), "alpha": math.radians(float(row["alpha_deg"]))}
        assert tables["cx"].evaluate(values) == pytest.approx(float(row["CX_body_x_forward"]), abs=1.0e-12)
        assert tables["cz"].evaluate(values) == pytest.approx(float(row["CZ_body_z_down"]), abs=1.0e-12)
    ####


def test_orion_thrust_and_mass_flow_tables_are_parser_valid() -> None:
    for motor in ("orion38", "orion50xl", "orion50sxl"):
        document = parse_table_file(ROOT / f"tables/{motor}_thrust_mdot.tbl")
        assert document.diagnostics == []
        tables = lower_tables(document)
        assert set(tables) == {"thrust", "mdot"}
        assert tables["thrust"].independent_variables == ("time",)
        assert tables["mdot"].independent_variables == ("time",)
        assert tables["thrust"].evaluate({"time": 0.0}) == 0.0
        assert tables["mdot"].evaluate({"time": 0.0}) == 0.0
    ####
