from pathlib import Path

import pytest

from taoryx.language.semantic_validation import validate_table_file
from taoryx.language.table_parser import parse_table_file, parse_table_text

FIXTURES = sorted(Path("examples/chapter03").glob("*.tbl")) + sorted(Path("examples/chapter04").glob("*.tbl"))

pytestmark = pytest.mark.table


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


def test_semantic_table_diagnostics_use_the_table_header_line() -> None:
    document = parse_table_text("\n(broken)\n table ca(mach)\n mach = 0 1\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "missing-dependent-values")
    assert diagnostic.location.line == 2
    assert document.tables[0].location.line == 2


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
    assert document.tables[0].source_text == "(demo) table output"
    assert operations[0].source_text == "label: add force(time) extrap"
    assert [assignment.source_text for assignment in operations[0].assignments] == ["time = 0 1", "force = 1 2"]
    assert operations[0].label == "label"
    assert operations[0].operand.name == "force"
    assert operations[0].operand.arguments == ["time"]
    assert operations[1].nested is not None
    assert operations[1].nested.source_text == "if (mach > 5.0) then add f2(time) no-extrap"
    assert operations[1].nested.operand.name == "f2"
    assert operations[2].operand == "saved"
    assert operations[3].operand == "done"
    assert not document.diagnostics


def test_store_and_goto_operations_require_name_operands() -> None:
    valid = parse_table_text(
        "(demo) table output\n"
        "start\n"
        "csto saved\n"
        "goto done\n"
        "done: end\n"
    )
    assert not valid.diagnostics

    invalid = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "csto 1\n"
        "goto target(time)\n"
        "end\n"
    )
    assert [item.code for item in invalid.diagnostics].count("invalid-operation-operand") == 2
    assert len(invalid.tables[0].operations) == 3


def test_full_table_storage_names_cannot_shadow_state_variables() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "csto alt\n"
        "end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "reserved-storage-variable")
    assert diagnostic.location.line == 3
    record = next(item for item in document.recovered_records if item.code == diagnostic.code)
    assert record.text == "csto alt"


def test_table_header_rejects_conflicting_extrapolation_options() -> None:
    document = parse_table_text(
        "(broken) table ca(mach) extrap no-extrap sref=5\n"
        "mach=0 1\n"
        "ca=0.1 0.2\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "conflicting-table-extrapolation")
    assert diagnostic.location.line == 1
    assert document.tables[0].options["extrapolation"] == "extrap"


def test_limited_state_tables_reject_undocumented_dependencies() -> None:
    document = parse_table_text(
        "(broken) table windv(thrust,time)\n"
        "thrust=0 1\n"
        "time=0 1\n"
        "windv=0 1 2 3\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "unsupported-limited-state-variable")
    assert diagnostic.location.line == 1
    assert any(record.code == "unsupported-limited-state-variable" for record in document.recovered_records)


def test_limited_state_tables_accept_user_defined_dependencies() -> None:
    document = parse_table_text(
        "(weight-cg) table cg(wt,config) no-extrap\n"
        "wt=1000,2000\n"
        "config=1,2\n"
        "cg=0.40,0.70,0.45,0.75\n"
    )

    assert not [item for item in document.diagnostics if item.code == "unsupported-limited-state-variable"]


def test_tables_cannot_depend_on_their_own_dependent_variable() -> None:
    document = parse_table_text(
        "(broken) table thrust(time,thrust)\n"
        "time=0 1\n"
        "thrust=0 1 2 3\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "self-referential-table")
    assert diagnostic.location.line == 1


def test_full_table_interpolation_group_preserves_and_diagnoses_reversed_independent_order() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "add cxo(alt,mach)\n"
        "mach=5 10\n"
        "alt=0 10000\n"
        "cxo=0.1 0.2 0.3 0.4\n"
        "end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "interpolation-assignment-order")
    assert diagnostic.location.line == 4
    assert document.tables[0].operations[0].assignments[0].name == "mach"


def test_full_table_recovers_multiple_operation_errors() -> None:
    document = parse_table_text("(broken) table output\nstart\nadd\nunknown\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "missing-operation-operand" in codes
    assert "unparsed-full-table-token" in codes
    assert "missing-full-table-end" in codes
    assert len(document.recovered_records) >= 3
    assert not document.executable_complete


def test_full_table_start_is_only_a_body_delimiter() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "start\n"
        "add value\n"
        "end\n"
    )

    assert any(item.code == "unparsed-full-table-token" for item in document.diagnostics)
    assert [operation.operator for operation in document.tables[0].operations] == ["add", "end"]


def test_malformed_table_input_returns_diagnostics_without_assertion_failure() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "if (value<) then add\n"
        "goto\n"
        "end\n"
    )

    assert document.diagnostics
    assert all(item.location is not None for item in document.diagnostics)
    assert all(record.location is not None for record in document.recovered_records)


def test_simple_table_assignment_named_start_is_not_a_full_table_delimiter() -> None:
    document = parse_table_text(
        "(start-value) table output(start)\n"
        "start=0,1\n"
        "output=2,3\n"
    )

    assert document.tables[0].format == "simple"
    assert [assignment.name for assignment in document.tables[0].assignments] == ["start", "output"]
    assert document.tables[0].assignments[0].source_text == "start=0,1"
    assert not document.diagnostics
    ####


def test_full_table_end_is_final_and_recovery_preserves_following_table() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "add 1\n"
        "end\n"
        "add 2\n"
        "(later) table output\n"
        "start\n"
        "end\n"
    )

    assert any(item.code == "trailing-full-table-text" for item in document.diagnostics)
    assert any(record.code == "trailing-full-table-text" for record in document.recovered_records)
    assert [table.name for table in document.tables] == ["broken", "later"]
    ####


def test_table_calls_require_identifier_arguments_and_reject_end_as_nested_operation() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "add force(alt,5)\n"
        "add other()\n"
        "if (mach > 5) then end\n"
        "end\n"
    )

    codes = [item.code for item in document.diagnostics]
    assert codes.count("invalid-table-call") == 2
    assert "invalid-if-operation" in codes
    assert [operation.operator for operation in document.tables[0].operations] == ["add", "add", "if", "end"]
    assert document.tables[0].operations[2].nested is None


def test_full_table_goto_accepts_value_labels_but_nested_if_is_rejected() -> None:
    valid = parse_table_text(
        "(labels) table output\n"
        "start\n"
        "1: add value\n"
        "goto 1\n"
        "end\n"
    )
    assert not valid.diagnostics
    assert valid.tables[0].operations[0].label == "1"
    assert valid.tables[0].operations[1].operand == "1"

    invalid = parse_table_text(
        "(nested) table output\n"
        "start\n"
        "if (mach > 5) then if (alt > 100) then add value\n"
        "end\n"
    )
    assert any(item.code == "invalid-if-operation" for item in invalid.diagnostics)
    assert any(record.code == "invalid-if-operation" for record in invalid.recovered_records)


def test_full_table_accepts_signed_numeric_labels_and_goto_destinations() -> None:
    document = parse_table_text(
        "(signed-labels) table output\n"
        "start\n"
        "-1.5: add value\n"
        "goto -1.5\n"
        "end\n"
    )

    assert not document.diagnostics
    assert document.tables[0].operations[0].label == "-1.5"
    assert document.tables[0].operations[1].operand == "-1.5"


def test_full_table_rejects_ambiguous_storage_and_control_flow_destinations() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "branch: add value\n"
        "branch: csto saved\n"
        "csto saved\n"
        "goto missing\n"
        "end\n"
    )

    codes = {item.code for item in document.diagnostics}
    assert {"duplicate-operation-label", "duplicate-storage-variable", "undefined-operation-label"} <= codes
    assert document.tables[0].operations[1].label == "branch"
    assert document.tables[0].operations[2].operand == "saved"
    assert all(record.location.line is not None for record in document.recovered_records)


def test_table_headers_and_labels_require_identifiers() -> None:
    simple = parse_table_text(
        "(bad$name) table output (mach 5)\n"
        "mach=0 1\n"
        "output=2 3\n"
    )
    simple_codes = [item.code for item in simple.diagnostics]
    assert "invalid-table-name" in simple_codes
    assert "invalid-independent-variable" in simple_codes

    empty = parse_table_text("(empty) table output ()\noutput=1 2\n")
    assert any(item.code == "empty-independent-variable-list" for item in empty.diagnostics)

    full = parse_table_text("(labels) table output\nstart\nbad$name: add value\nend\n")
    assert any(item.code == "invalid-operation-label" for item in full.diagnostics)
    assert full.tables[0].operations[0].label == "bad$name"


def test_table_option_values_are_atoms_and_documented_unit_atoms_survive() -> None:
    valid = parse_table_text("(units) table mdot(time) units=lb/sec\ntime=0 1\nmdot=1 2\n")
    assert not valid.diagnostics
    assert valid.tables[0].options["units"] == "lb/sec"

    invalid = parse_table_text(
        "(broken) table output units=(bad)\n"
        "output=1 2\n"
        "(later) table ca\n"
        "ca=3 4\n"
    )
    assert any(item.code == "invalid-table-option-value" for item in invalid.diagnostics)
    assert [table.name for table in invalid.tables] == ["broken", "later"]


def test_table_header_parameters_follow_documented_table_type_rules() -> None:
    valid = parse_table_text(
        "(ca-table) table ca(mach) sref=2.5\n"
        "mach=0 1\n"
        "ca=0.1 0.2\n"
        "(motor) table thrust(time) units=n\n"
        "time=0 1\n"
        "thrust=1 2\n"
        "(flow) table mdot(time) units=kg/hr\n"
        "time=0 1\n"
        "mdot=1 2\n"
    )
    assert not valid.diagnostics

    invalid = parse_table_text(
        "(bad) table output units=lb sref=abc\n"
        "output=1\n"
        "(bad-coeff) table ca(mach) sref=abc\n"
        "mach=0\n"
        "ca=1\n"
        "(bad-flow) table mdot(time) units=lb\n"
        "time=0\n"
        "mdot=1\n"
    )
    codes = {item.code for item in invalid.diagnostics}
    assert "unsupported-table-option" in codes
    assert "unsupported-table-units" in codes
    assert "invalid-table-sref" in codes


def test_table_assignment_names_require_identifiers_and_recover_to_next_table() -> None:
    document = parse_table_text(
        "(broken) table ca\n"
        "5=0 1\n"
        "ca=2 3\n"
        "(later) table ca\n"
        "ca=4 5\n"
    )

    assert any(item.code == "invalid-table-assignment-name" for item in document.diagnostics)
    assert [table.name for table in document.tables] == ["broken", "later"]
    assert document.tables[0].assignments[0].name == "5"
    assert document.tables[1].assignments[0].values == [4.0, 5.0]


def test_recovered_table_text_preserves_indentation_and_comments() -> None:
    document = parse_table_text("(broken) table ca\n  invalid-token # retain this source\n")

    assert any(record.text == "  invalid-token # retain this source" for record in document.recovered_records)


def test_table_omissions_are_recovered_as_documentation_source() -> None:
    document = parse_table_text(
        "(excerpt) table output\n"
        "output=0 1\n"
        "  . . . # omitted by the manual\n"
    )

    assert not document.executable_complete
    record = next(item for item in document.recovered_records if item.code == "documentation-excerpt")
    assert record.text == "  . . . # omitted by the manual"
    assert record.location.line == 3


def test_table_diagnostics_are_returned_in_source_order_after_semantic_validation() -> None:
    document = parse_table_text(
        "(first) table ca(mach)\n"
        "mach=0 1 1\n"
        "ca=0.1 0.2 0.3\n"
        "(second) table output\n"
        "unparsed-token\n"
    )

    lines = [diagnostic.location.line for diagnostic in document.diagnostics if diagnostic.location is not None]
    assert lines == sorted(lines)
    assert lines[0] == 2


def test_duplicate_table_diagnostic_is_case_insensitive_and_points_to_duplicate_header() -> None:
    document = parse_table_text("(Same) table ca\nca=1\n(same) table ca\nca=2\n")

    diagnostic = next(item for item in validate_table_file(document) if item.code == "duplicate-table")
    assert diagnostic.location is not None
    assert diagnostic.location.line == 3


def test_full_table_if_requires_simple_relation_and_then() -> None:
    document = parse_table_text(
        "(broken) table output\n"
        "start\n"
        "if (mach > 5 && alt < 50000) add value\n"
        "end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-if-condition" in codes
    assert "missing-if-then" in codes
    assert document.tables[0].operations[0].nested is not None


def test_skewed_manual_groups_accept_leading_decimal_values() -> None:
    document = parse_table_text(
        "(skewed)\n"
        "table output\n"
        "start\n"
        "add cxo(alt,mach)\n"
        "alt=0,50000\n"
        "mach=3,5,7,9\n"
        "cxo=.020,.021,.019,.018\n"
        "     .021,.022,.021,.020\n"
        "end\n"
    )

    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity.value == "error"]


def test_simple_table_independent_values_must_be_strictly_monotonic() -> None:
    duplicate = parse_table_text(
        "(duplicate) table ca(mach)\n"
        "mach=0,1,1\n"
        "ca=0.1,0.2,0.3\n"
    )
    unordered = parse_table_text(
        "(unordered) table ca(mach)\n"
        "mach=0,2,1\n"
        "ca=0.1,0.2,0.3\n"
    )

    assert any(item.code == "duplicate-independent-values" for item in duplicate.diagnostics)
    assert any(item.code == "unordered-independent-values" for item in unordered.diagnostics)
    assert all(item.severity.value == "error" for item in duplicate.diagnostics if "independent" in item.code)
    ####


def test_duplicate_table_assignments_are_diagnosed_without_overwriting_source_data() -> None:
    document = parse_table_text(
        "(duplicate) table ca(mach)\n"
        "mach=0,1\n"
        "mach=2,3\n"
        "ca=0.1,0.2\n"
    )

    assert any(item.code == "duplicate-table-assignment" for item in document.diagnostics)
    assert [assignment.values for assignment in document.tables[0].assignments[:2]] == [[0.0, 1.0], [2.0, 3.0]]
    ####


def test_duplicate_full_table_interpolation_assignments_are_diagnosed() -> None:
    document = parse_table_text(
        "(duplicate) table output\n"
        "start\n"
        "add factor(mach)\n"
        "mach=0,1\n"
        "mach=2,3\n"
        "factor=1,2\n"
        "end\n"
    )

    assert any(item.code == "duplicate-interpolation-assignment" for item in document.diagnostics)
    assert len(document.tables[0].operations[0].assignments) == 3
    ####


def test_tables_and_calls_limit_independent_variables_to_five() -> None:
    document = parse_table_text(
        "(too-wide) table output(a,b,c,d,e,f)\n"
        "a=0\n"
        "b=0\n"
        "c=0\n"
        "d=0\n"
        "e=0\n"
        "f=0\n"
        "output=1\n"
        "(full) table output\n"
        "start\n"
        "add value(a,b,c,d,e,f)\n"
        "end\n"
    )

    codes = {item.code for item in document.diagnostics}
    assert "too-many-independent-variables" in codes
    assert "too-many-table-call-arguments" in codes
    assert all(record.location is not None for record in document.recovered_records)
    ####
