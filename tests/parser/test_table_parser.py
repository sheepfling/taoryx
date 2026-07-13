from pathlib import Path

import pytest

from taoryx.language.semantic_validation import validate_table_file
from taoryx.language.table_parser import parse_table_file, parse_table_text

FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(Path("examples/chapter04").glob("*.tbl"))


@pytest.mark.parametrize("path", FIXTURES)
def test_manual_table_fixture_parses(path: Path) -> None:
    document = parse_table_file(path)
    assert document.tables
    assert not [item for item in validate_table_file(document) if item.severity == "error"]
####


def test_table_recovery_continues_at_next_table_identifier() -> None:
    document = parse_table_text("not a table\n(good)\n table ca(mach)\n mach = 0 1\n ca = 1 2\n")

    assert [table.name for table in document.tables] == ["good"]
    assert any(diagnostic.code == "missing-table-name" for diagnostic in document.diagnostics)
    assert document.recovered_records[0].text == "not a table"


def test_table_diagnostic_preserves_token_column() -> None:
    document = parse_table_text("  not a table\n(good)\n table ca(mach)\n mach = 0 1\n ca = 1 2\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "missing-table-name")
    assert diagnostic.location is not None
    assert diagnostic.location.line == 1
    assert diagnostic.location.column == 3


def test_full_table_operations_preserve_calls_conditionals_labels_and_end() -> None:
    document = parse_table_text(
        "(demo) table output\n"
        "start\n"
        "label: add force(time) extrap\n"
        "time = 0 1\n"
        "force = 1 2\n"
        "if (mach > 5.0) then add f2(time) no-extrap\n"
        "csto saved\n"
        "goto done\n"
        "done: end\n"
    )

    operations = document.tables[0].operations
    assert [operation.operator for operation in operations] == ["add", "if", "csto", "goto", "end"]
    assert operations[0].label == "label"
    assert operations[0].operand.name == "force"
    assert operations[0].operand.arguments == ["time"]
    assert operations[1].nested is not None
    assert operations[1].nested.operand.name == "f2"
    assert operations[2].operand == "saved"
    assert operations[3].operand == "done"
    assert not document.diagnostics


def test_full_table_recovers_multiple_operation_errors() -> None:
    document = parse_table_text("(broken) table output\nstart\nadd\nunknown\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "missing-operation-operand" in codes
    assert "unparsed-full-table-token" in codes
    assert "missing-full-table-end" in codes
    assert len(document.recovered_records) >= 3
    assert not document.executable_complete
