from pathlib import Path

from taoryx.language.ingest import FileKind, ingest_file

FIXTURES = Path(__file__).parents[1] / "fixtures" / "grammar_baseline"


def test_independent_valid_table_fixture_is_accepted_losslessly() -> None:
    result = ingest_file(FIXTURES / "valid.tbl")

    assert result.kind is FileKind.TABLE
    assert result.valid
    assert result.source.render_bytes() == (FIXTURES / "valid.tbl").read_bytes()
    assert [table.name for table in result.document.tables] == ["demo-ca", "demo-output"]


def test_independent_valid_problem_fixture_is_accepted_losslessly() -> None:
    result = ingest_file(FIXTURES / "valid.prb")

    assert result.kind is FileKind.PROBLEM
    assert result.valid
    assert result.source.render_bytes() == (FIXTURES / "valid.prb").read_bytes()
    assert result.document.problems[0].trajectories[0].segments[0].number == 1


def test_independent_invalid_table_fixture_reports_multiple_recoverable_errors() -> None:
    result = ingest_file(FIXTURES / "invalid.tbl")

    codes = {diagnostic.code for diagnostic in result.diagnostics}
    assert {"invalid-if-condition", "undefined-operation-label"} <= codes
    assert len(result.diagnostics) >= 2
    assert all(record.location is not None for record in result.document.recovered_records)


def test_independent_invalid_problem_fixture_reports_multiple_source_located_errors() -> None:
    result = ingest_file(FIXTURES / "invalid.prb")

    codes = {diagnostic.code for diagnostic in result.diagnostics}
    assert {"missing-initial-mass", "unknown-goto-segment", "invalid-when-condition", "invalid-when-statement"} <= codes
    assert len(result.diagnostics) >= 4
    assert all(diagnostic.location is not None for diagnostic in result.diagnostics)
    assert all(record.location is not None for record in result.document.recovered_records)
