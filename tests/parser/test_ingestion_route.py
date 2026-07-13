from pathlib import Path

import pytest

from taoryx.language.ingest import FileKind, UnsupportedFileKindError, ingest_file, ingest_text, kind_for_path
from taoryx.language.table_parser import parse_table_text, table_type_catalog, table_variable_catalog

BASELINE_FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(
    Path("examples/chapter04").glob("*.prb")
) + sorted(Path("examples/chapter04").glob("*.tbl"))


@pytest.mark.parametrize("path", BASELINE_FIXTURES, ids=lambda path: str(path))
def test_ingest_route_preserves_and_validates_baseline(path: Path) -> None:
    result = ingest_file(path)

    assert result.kind is (FileKind.PROBLEM if path.suffix == ".prb" else FileKind.TABLE)
    assert result.source.render_bytes() == path.read_bytes()
    assert result.valid


def test_kind_detection_and_unsupported_extension() -> None:
    assert kind_for_path("example.prb") is FileKind.PROBLEM
    assert kind_for_path("example.tbl") is FileKind.TABLE
    with pytest.raises(UnsupportedFileKindError):
        kind_for_path("example.txt")


def test_available_table_references_are_case_insensitive() -> None:
    result = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=(STAGE1)\n"
        "*when time>1 stop\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        available_tables={"stage1"},
    )

    assert not [item for item in result.diagnostics if item.code == "external-table-reference"]


def test_typed_table_catalog_rejects_a_table_used_for_the_wrong_role() -> None:
    result = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=(stage1)\n"
        "*when time>1 stop\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        available_tables={"stage1": "cd"},
    )

    diagnostic = next(item for item in result.diagnostics if item.code == "table-type-mismatch")
    assert diagnostic.location is not None
    assert diagnostic.location.line == 5


def test_semantic_diagnostic_gets_source_preserving_recovery_record() -> None:
    result = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*when time>1 goto 9 # unresolved target\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        source_path="demo.prb",
    )

    diagnostic = next(item for item in result.diagnostics if item.code == "unknown-goto-segment")
    record = next(item for item in result.document.recovered_records if item.code == "unknown-goto-segment")
    assert diagnostic.location == record.location
    assert record.text == "*when time>1 goto 9 # unresolved target"


def test_unified_diagnostics_and_recovery_are_source_ordered() -> None:
    result = ingest_text(
        "(demo) table ca\n"
        "ca=1\n"
        "(demo) table cd\n"
        "cd=2\n"
        "(broken) table unknown\n"
        "unknown=1\n",
        kind=FileKind.TABLE,
        source_path="ordered.tbl",
    )

    assert [(item.code, item.location.line) for item in result.diagnostics] == [
        ("duplicate-table", 3),
        ("unknown-table-type", 5),
    ]
    assert [(item.code, item.location.line) for item in result.document.recovered_records] == [
        ("duplicate-table", 3),
        ("unknown-table-type", 5),
    ]
    assert result.valid is False
    ####


def test_table_type_catalog_is_case_folded_and_omits_ambiguous_names() -> None:
    document = parse_table_text(
        "(Stage1) table ca\n"
        "ca=1\n"
        "(stage1) table cd\n"
        "cd=2\n"
        "(stage2) table thrust\n"
        "thrust=3\n"
    )

    assert table_type_catalog(document) == {"stage2": "thrust"}


def test_table_variable_catalog_validates_only_provably_unused_user_variables() -> None:
    table_document = parse_table_text(
        "(ca-table) table ca(mach,flaps)\n"
        "mach=0,1\n"
        "flaps=0,1\n"
        "ca=1,2,3,4\n"
    )
    table_types = table_type_catalog(table_document)
    table_variables = table_variable_catalog(table_document)
    valid = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=(ca-table) flaps=25\n"
        "*when time>1 stop\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        available_tables=table_types,
        available_table_variables=table_variables,
    )
    invalid = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=(ca-table) gear=0\n"
        "*when time>1 stop\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        available_tables=table_types,
        available_table_variables=table_variables,
    )

    assert not [item for item in valid.diagnostics if item.code == "unknown-table-variable-assignment"]
    diagnostic = next(item for item in invalid.diagnostics if item.code == "unknown-table-variable-assignment")
    assert diagnostic.location is not None
    assert diagnostic.location.line == 5


@pytest.mark.parametrize("block", ["constants", "aero", "cg", "prop"])
def test_user_variable_assignments_cannot_shadow_documented_state_variables(block: str) -> None:
    assignment = {
        "constants": "alt=1",
        "aero": "ca=1 alt=2",
        "cg": "cg=1 alt=2",
        "prop": "thrust=1 alt=2",
    }[block]
    result = ingest_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        f"*{block} {assignment}\n"
        "*when time>1 stop\n"
        "*end\n",
        kind=FileKind.PROBLEM,
        source_path="reserved.prb",
    )

    diagnostic = next(item for item in result.diagnostics if item.code == "reserved-user-variable")
    assert diagnostic.location is not None
    assert diagnostic.location.line == 5
    record = next(item for item in result.document.recovered_records if item.code == diagnostic.code)
    assert record.text == f"*{block} {assignment}"
    assert result.valid is False


def test_problem_rejects_more_than_five_optimization_loops_with_recovery() -> None:
    text = "(demo)\n" + "".join(
        f"*optimize {letter} for vel=max on segment 1, trajectory 1\npar-1=1\n"
        for letter in "abcdef"
    ) + "*end\n"

    result = ingest_text(text, kind=FileKind.PROBLEM, source_path="too-many-loops.prb")

    diagnostic = next(item for item in result.diagnostics if item.code == "too-many-optimize-loops")
    assert diagnostic.location.line == 12
    record = next(item for item in result.document.recovered_records if item.code == diagnostic.code)
    assert record.text == "*optimize f for vel=max on segment 1, trajectory 1"
