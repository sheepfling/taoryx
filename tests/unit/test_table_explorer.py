from pathlib import Path

import pytest

from taoryx.language.table_parser import parse_table_text
from taoryx.runtime.cli import main
from taoryx.table_explorer import TableInspectionStatus, explain_interpolation, inspect_table_document, inspect_table_file

ROOT = Path(__file__).resolve().parents[1] / "fixtures" / "table_examples_v1"

pytestmark = pytest.mark.table


def test_regular_table_inspection_exposes_catalog_and_prepared_runtime() -> None:
    artifact = inspect_table_file(ROOT / "aero" / "clean-cd.tbl")

    table = artifact.tables[0]
    assert table.status is TableInspectionStatus.PREPARED
    assert table.shape == (3, 3)
    assert table.independent_variables == ("mach", "alpha")
    assert table.prepared is not None
    assert artifact.catalog()[0]["format"] == "regular-grid"
    assert artifact.to_dict()["valid"] is True


def test_full_table_is_source_only_and_retains_operations() -> None:
    artifact = inspect_table_document(
        parse_table_text("(demo) table output\nstart\nadd force(time)\ntime=0 1\nforce=2 3\nend\n", "demo.tbl")
    )

    table = artifact.tables[0]
    assert table.status is TableInspectionStatus.SOURCE_ONLY
    assert table.format.value == "full-program"
    assert [operation.operator for operation in table.operations] == ["add", "end"]


def test_partial_table_remains_inspectable_without_preparation() -> None:
    artifact = inspect_table_document(parse_table_text("(excerpt) table ca(mach)\nmach=0 1\nca=0.1\n", "excerpt.tbl"))

    table = artifact.tables[0]
    assert table.status is TableInspectionStatus.INVALID
    assert table.assignments[0].source_text == "mach=0 1"
    assert artifact.format_catalog().startswith("excerpt.tbl: 1 table(s)")


def test_invalid_table_diagnostics_are_source_located() -> None:
    artifact = inspect_table_document(parse_table_text("(bad) table ca(mach)\nmach=0 1 0\nca=1 2 3\n", "bad.tbl"))

    assert artifact.tables[0].status is TableInspectionStatus.INVALID
    assert any(diagnostic.location and diagnostic.location.line == 2 for diagnostic in artifact.diagnostics)


def test_interpolation_explanation_matches_runtime_value() -> None:
    artifact = inspect_table_file(Path("examples/chapter03/stmi-simple.tbl"))

    explanation = explain_interpolation(artifact.table(), {"mach": 1.14})

    assert explanation.status == "interpolated"
    assert explanation.brackets[0].lower == pytest.approx(1.1)
    assert explanation.brackets[0].upper == pytest.approx(1.18)
    assert explanation.brackets[0].fraction == pytest.approx(0.5)
    assert explanation.value == pytest.approx(1.77)
    assert sum(float(corner["weight"]) for corner in explanation.corners) == pytest.approx(1.0)


def test_duplicate_axis_table_remains_source_inspectable() -> None:
    artifact = inspect_table_file(Path("examples/chapter03/orbus-mdt.tbl"))

    assert artifact.tables[0].operations
    assert artifact.tables[0].prepared is None
    assert artifact.tables[0].status in {TableInspectionStatus.INVALID, TableInspectionStatus.SOURCE_ONLY}


def test_table_inspect_cli_writes_standalone_html(tmp_path: Path) -> None:
    output = tmp_path / "stmi.html"

    assert main(["table", "inspect", "examples/chapter03/stmi-simple.tbl", "--at", "mach=1.14", "--html", str(output)]) == 0
    assert "TAORYX Table Explorer" in output.read_text(encoding="utf-8")
    assert "data:image/png;base64" in output.read_text(encoding="utf-8")
