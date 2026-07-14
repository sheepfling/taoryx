import pytest

from taoryx.language.grammar_contracts import SUPPORTED_FLY_GUIDANCE_RULES
from taoryx.language.ingest import FileKind, ingest_text
from taoryx.language.models import OptimizeBlock
from taoryx.language.problem_parser import parse_problem_text


def test_end_closes_one_problem_and_allows_the_next_problem() -> None:
    document = parse_problem_text("(first)\n*title first\n*end\n(second)\n*title second\n*end\n")

    assert [problem.name for problem in document.problems] == ["first", "second"]
    assert all(problem.ended for problem in document.problems)
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_block_after_end_is_not_attached_to_the_closed_problem() -> None:
    document = parse_problem_text("(demo)\n*end\n*title invalid\n")

    assert [block.keyword for block in document.problems[0].blocks] == []
    assert any(diagnostic.code == "block-after-end" for diagnostic in document.diagnostics)


def test_end_before_a_problem_is_an_error() -> None:
    document = parse_problem_text("*end\n")

    assert {diagnostic.code for diagnostic in document.diagnostics} >= {"end-before-problem", "missing-problem"}


def test_invalid_problem_name_is_diagnosed_without_hiding_later_problems() -> None:
    document = parse_problem_text("(bad name!)\n*end\n(valid)\n*end\n")

    assert [problem.name for problem in document.problems] == ["bad name!", "valid"]
    assert document.problems[1].ended
    diagnostic = next(item for item in document.diagnostics if item.code == "invalid-problem-name")
    assert diagnostic.location.line == 1
    assert any(record.code == "invalid-problem-name" for record in document.recovered_records)


def test_missing_end_is_diagnosed_at_eof_and_preserves_the_problem() -> None:
    document = parse_problem_text("(demo)\n*title incomplete\n")

    assert document.problems[0].ended is False
    diagnostic = next(item for item in document.diagnostics if item.code == "missing-end")
    assert diagnostic.location.line == 2
    assert any(record.code == "missing-end" and record.text == "*title incomplete" for record in document.recovered_records)


def test_missing_end_before_next_problem_does_not_hide_later_problems() -> None:
    document = parse_problem_text("(first)\n*title incomplete\n(second)\n*end\n")

    assert [problem.name for problem in document.problems] == ["first", "second"]
    assert any(diagnostic.code == "missing-end" for diagnostic in document.diagnostics)
    assert document.problems[1].ended is True


def test_duplicate_trajectory_number_is_diagnosed_and_preserved() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*trajectory 1 target start on 1\n"
        "*segment 1\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "duplicate-trajectory-number")
    assert diagnostic.location.line == 4
    assert len(document.problems[0].trajectories) == 2
    assert any(record.code == "duplicate-trajectory-number" for record in document.recovered_records)
    ####


def test_duplicate_trajectory_name_is_diagnosed_case_insensitively() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 Vehicle start on 1\n"
        "*trajectory 2 vehicle start on 1\n"
        "*end\n"
    )

    assert any(item.code == "duplicate-trajectory-name" for item in document.diagnostics)
    assert any(record.code == "duplicate-trajectory-name" for record in document.recovered_records)
    ####


def test_duplicate_segment_number_is_scoped_to_its_trajectory() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1 first\n"
        "*segment 1 duplicate\n"
        "*trajectory 2 target start on 1\n"
        "*segment 1 allowed-in-new-trajectory\n"
        "*end\n"
    )

    duplicate = next(item for item in document.diagnostics if item.code == "duplicate-segment-number")
    assert duplicate.location.line == 4
    assert len(document.problems[0].trajectories[0].segments) == 2
    assert len(document.problems[0].trajectories[1].segments) == 1
    assert any(record.code == "duplicate-segment-number" for record in document.recovered_records)
    ####


def test_reset_and_increment_enforce_state_coordinate_restrictions() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*increment wt=1 mass=2 alt=1 xecfc=2\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"conflicting-reset-mass", "mixed-reset-coordinate-systems"} <= codes
    assert len(document.problems[0].trajectories[0].segments[0].blocks[0].assignments) == 4
    assert {record.code for record in document.recovered_records} >= {
        "conflicting-reset-mass",
        "mixed-reset-coordinate-systems",
    }
    ####


def test_absolute_time_reset_is_rejected_for_multiple_trajectories() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 first start on 1\n"
        "*segment 1\n"
        "*reset time=0\n"
        "*when time>1 stop\n"
        "*trajectory 2 second start on 1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "absolute-time-discontinuity-multiple-trajectories")
    assert diagnostic.location.line == 4
    assert any(record.code == diagnostic.code for record in document.recovered_records)
    ####


def test_reset_and_increment_allow_consistent_coordinate_families() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*increment alt=1 latgd=2 long=3 gamgd=4 psigd=5\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [
        item
        for item in document.diagnostics
        if item.code in {"conflicting-reset-mass", "mixed-reset-coordinate-systems"}
    ]
    ####


def test_segment_requires_at_least_one_when_final_condition() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1 no-termination\n"
        "*aero ca=(drag)\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "missing-segment-when")
    assert diagnostic.location.line == 3
    assert any(record.code == "missing-segment-when" for record in document.recovered_records)
    ####


def test_segment_with_when_final_condition_is_accepted() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1 terminates\n"
        "*when time > 1 stop\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code == "missing-segment-when"]
    ####


def test_trajectory_requires_initial_and_segment_children() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*end\n")

    codes = {item.code for item in document.diagnostics}
    assert {"missing-trajectory-initial", "missing-trajectory-segment"} <= codes
    assert all(record.code in codes for record in document.recovered_records)
    ####


def test_trajectory_with_initial_segment_and_when_is_complete() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial geodetic\n"
        "long=0 lat=0 alt=0 vel=1 gama=0 psi=0 time=0 wt=1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.severity.value == "error"]
    ####


def test_define_assignment_can_span_lines_until_semicolon() -> None:
    document = parse_problem_text("(demo)\n*define x\nvalue = alt +\n  3.0D+02;\n*end\n")

    assignment = document.problems[0].blocks[0].assignments[0]
    assert assignment.name == "value"


def test_define_accepts_the_manual_function_vocabulary() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*define value\n"
        "value = atan2(sinh(x), cosh(x)) + ceil(floor(abs(log10(table(out))))) + max(x);\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]
    ####


def test_define_unknown_function_is_diagnosed_and_preserved() -> None:
    document = parse_problem_text("(demo)\n*define value\nvalue = mystery(alt);\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "unsupported-define-function")
    assert diagnostic.location.line == 3
    assert document.problems[0].blocks[0].assignments[0].name == "value"
    assert any(record.code == "unsupported-define-function" for record in document.recovered_records)
    ####


def test_define_recovery_preserves_physical_indentation_and_comments() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define value\n"
        "  value = mystery(alt); # retain this source\n"
        "*end\n"
    )

    recovered = next(record for record in document.recovered_records if record.code == "unsupported-define-function")
    assert recovered.text == "  value = mystery(alt); # retain this source"
    ####


def test_synthesized_optimize_and_search_recovery_uses_physical_source_lines() -> None:
    optimize = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "  constrain vel # retain optimize source\n"
        "*end\n"
    )
    optimize_record = next(record for record in optimize.recovered_records if record.code == "invalid-optimize-constraint")
    assert optimize_record.text == "  constrain vel # retain optimize source"

    search = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alt until alt # retain search source\n"
        "*end\n"
    )
    search_record = next(record for record in search.recovered_records if record.code == "invalid-search-objective")
    assert search_record.text == "*search 1 vary alt until alt # retain search source"
    ####


def test_define_function_arity_and_table_argument_are_checked() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*define value\n"
        "value = sqrt(alt, vel) + table(a, b);\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("invalid-define-function-arity") == 1
    assert "invalid-define-table-argument" not in codes
    assert all(record.code == "invalid-define-function-arity" for record in document.recovered_records)
    ####


def test_define_table_function_requires_a_table_name_argument() -> None:
    document = parse_problem_text("(demo)\n*define value\nvalue = table(alt + 1);\n*end\n")

    assert any(diagnostic.code == "invalid-define-table-argument" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-define-table-argument" for record in document.recovered_records)
    ####


def test_problem_define_cannot_evaluate_a_table() -> None:
    document = parse_problem_text("(demo)\n*define value\nvalue = table(out);\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "table-call-outside-trajectory")
    assert diagnostic.location.line == 3
    assert document.problems[0].blocks[0].assignments[0].value.kind == "call"
    assert any(record.code == "table-call-outside-trajectory" for record in document.recovered_records)
    ####


def test_trajectory_define_can_evaluate_a_table() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*define value\n"
        "value = table(out);\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code == "table-call-outside-trajectory"]
    assert document.problems[0].trajectories[0].blocks[0].assignments[0].value.kind == "call"
    ####


def test_define_diagnoses_a_temporary_used_before_assignment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*define result\n"
        "result = temporary + alt;\n"
        "temporary = 1;\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "define-variable-used-before-assignment")
    assert diagnostic.location.line == 4
    assert any(record.code == "define-variable-used-before-assignment" for record in document.recovered_records)
    ####


def test_define_accepts_a_temporary_after_it_is_assigned() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*define result\n"
        "temporary = 1;\n"
        "result = temporary + alt;\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code == "define-variable-used-before-assignment"]
    ####


def test_ordinary_block_assignments_reject_function_calls_but_keep_table_references() -> None:
    invalid = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*initial from trajectory 1, segment 1\n*segment 1\n*aero ca=lookup(mach)\n*when time>1 stop\n*end\n")
    assert any(diagnostic.code == "unsupported-assignment-call" for diagnostic in invalid.diagnostics)
    assert any(record.code == "unsupported-assignment-call" for record in invalid.recovered_records)

    body = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero\n"
        "ca=lookup(mach)\n"
        "*when time>1 stop\n"
        "*end\n"
    )
    assert any(record.code == "unsupported-assignment-call" for record in body.recovered_records)

    valid = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*initial from trajectory 1, segment 1\n*segment 1\n*aero ca=(lookup)\n*when time>1 stop\n*end\n")
    assert not valid.diagnostics
    ####


def test_ordinary_block_assignments_reject_undocumented_operators_without_overwriting_source() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*integ dt+=1 dt<=2\n*when time>1 stop\n*end\n")

    block = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert [assignment.operator for assignment in block.assignments] == ["+=", "<="]
    assert sum(diagnostic.code == "unsupported-assignment-operator" for diagnostic in document.diagnostics) == 2
    assert sum(record.code == "unsupported-assignment-operator" for record in document.recovered_records) == 2
    ####


def test_fixed_numeric_parameters_reject_nonnumeric_values_without_guessing() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*earth wgs-84 omega=rotation\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*integ dt=step\n"
        "*radar 1 station alt=(altitude)\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    parameters = [diagnostic for diagnostic in document.diagnostics if diagnostic.code == "nonnumeric-block-parameter"]
    assert {diagnostic.location.line for diagnostic in parameters} == {2, 5, 6}
    assert len([record for record in document.recovered_records if record.code == "nonnumeric-block-parameter"]) == 3
    assert document.problems[0].blocks[0].assignments[0].value.kind == "name"
    ####


def test_fixed_parameter_duplicates_are_preserved_and_diagnosed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*iip iip_beta=1 iip_beta=2\n"
        "*segment 1\n"
        "*integ dt=0.1\n"
        "dt=0.2\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("duplicate-block-parameter") == 2
    assert len([record for record in document.recovered_records if record.code == "duplicate-block-parameter"]) == 2
    ####


def test_conditions_and_limits_reject_function_calls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*limits alpha<sin(alt)\n"
        "*when mystery(alt)>0 stop\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "unsupported-limit-call" in codes
    assert "unsupported-condition-call" in codes
    assert any(record.code == "unsupported-limit-call" for record in document.recovered_records)
    assert any(record.code == "unsupported-condition-call" for record in document.recovered_records)
    ####


def test_limits_reject_guidance_rules_excluded_by_the_manual() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*limits intercept<1 propnav<2 downria<3 upria<4 l/d-max<5\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    limits = document.problems[0].trajectories[0].segments[0].blocks[0].limits
    assert [limit.variable for limit in limits] == ["intercept", "propnav", "downria", "upria", "l/d-max"]
    diagnostics = [item for item in document.diagnostics if item.code == "unlimit-able-guidance-variable"]
    assert len(diagnostics) == 5
    assert all(item.location.line == 4 for item in diagnostics)
    assert len([record for record in document.recovered_records if record.code == "unlimit-able-guidance-variable"]) == 5
    ####


def test_fly_blocks_have_manual_count_and_wildcard_restrictions() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*fly yawgd=*\n"
        "*fly pitchgd=*\n"
        "*fly rollgd=*\n"
        "*fly alpha=1\n"
        "*fly beta=2\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    codes = [item.code for item in document.diagnostics]
    assert codes.count("wildcard-fly-in-first-segment") == 3
    assert codes.count("too-many-fly-blocks") == 1
    fly_blocks = document.problems[0].trajectories[0].segments[0].blocks[:5]
    assert all(block.keyword == "fly" for block in fly_blocks)
    assert all(any(record.code == code for record in document.recovered_records) for code in {"wildcard-fly-in-first-segment", "too-many-fly-blocks"})
    ####


def test_fly_wildcard_is_allowed_after_the_first_segment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*when time>1 goto 2\n"
        "*segment 2\n"
        "*fly yawgd=*\n"
        "*when time>2 stop\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.code == "wildcard-fly-in-first-segment"]
    ####


def test_intercept_and_propnav_consume_two_fly_controls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*fly intercept=2\n"
        "*fly alpha=1\n"
        "*fly beta=2\n"
        "*fly power=3\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "too-many-special-fly-blocks")
    assert diagnostic.location.line == 7
    assert any(record.code == "too-many-special-fly-blocks" for record in document.recovered_records)
    ####


def test_optimize_constraints_accept_signed_numeric_endpoints() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain gamgd=-80\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.constraints[0].right.text == "-80"
    assert not document.diagnostics


def test_define_control_statement_is_typed_and_preserved() -> None:
    document = parse_problem_text("(demo)\n*define value\nif (alt > 100) then value = 1;\nelse value = 0;\n*end\n")

    block = document.problems[0].blocks[0]
    assert [control.kind for control in block.control_statements] == ["if", "else"]
    assert block.control_statements[0].assignment is not None
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_define_orphan_else_is_diagnosed_without_false_control_semantics() -> None:
    document = parse_problem_text("(demo)\n*define value\nelse value = 0;\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "orphan-define-else")
    assert diagnostic.location.line == 3
    assert document.problems[0].blocks[0].control_statements[0].kind == "else"
    assert any(record.code == "orphan-define-else" and record.text == "else value = 0;" for record in document.recovered_records)


def test_define_if_control_accepts_documented_c_style_assignment() -> None:
    document = parse_problem_text("(demo)\n*define value\nif (alt > 100) value = 1;\n*end\n")

    control = document.problems[0].blocks[0].control_statements[0]
    assert control.kind == "if"
    assert control.assignment is not None
    assert control.assignment.name == "value"
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


@pytest.mark.parametrize(
    ("function", "arguments"),
    [
        ("abs", "alt"),
        ("acos", "alt"),
        ("asin", "alt"),
        ("atan", "alt"),
        ("atan2", "alt, vel"),
        ("ceil", "alt"),
        ("cos", "alt"),
        ("cosh", "alt"),
        ("exp", "alt"),
        ("floor", "alt"),
        ("log", "alt"),
        ("log10", "alt"),
        ("sin", "alt"),
        ("sinh", "alt"),
        ("sqrt", "alt"),
        ("tan", "alt"),
        ("tanh", "alt"),
        ("table", "output-table"),
    ],
)
def test_define_accepts_every_documented_function_form(function: str, arguments: str) -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*define value\n"
        f"value = {function}({arguments});\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [item for item in document.diagnostics if item.severity.value == "error"], function


def test_define_braced_if_else_preserves_typed_branches() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define var\n"
        "if (mach > 1.0) {\n"
        "  x = 1 + mach*mach;\n"
        "  var = sqrt(x);\n"
        "} else {\n"
        "  var = 0;\n"
        "}\n"
        "*end\n"
    )

    control = document.problems[0].blocks[0].control_statements[0]
    assert [assignment.name for assignment in control.body] == ["x", "var"]
    assert [assignment.name for assignment in control.else_body] == ["var"]
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]


def test_define_braced_nested_controls_are_typed_in_their_parent_branch() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define value\n"
        "if (mach > 1.0) {\n"
        "  if (alt > 100) {\n"
        "    value = mach;\n"
        "  } else {\n"
        "    value = 0;\n"
        "  }\n"
        "} else {\n"
        "  value = -1;\n"
        "}\n"
        "*end\n"
    )

    control = document.problems[0].blocks[0].control_statements[0]
    assert len(control.body_controls) == 1
    nested = control.body_controls[0]
    assert [assignment.name for assignment in nested.body] == ["value"]
    assert [assignment.name for assignment in nested.else_body] == ["value"]
    assert [assignment.name for assignment in control.else_body] == ["value"]
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]
    ####


def test_define_unclosed_brace_is_diagnosed_and_recovered() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define var\n"
        "if (mach > 1.0) {\n"
        "  var = mach;\n"
        "*end\n"
    )

    assert any(diagnostic.code == "unclosed-define-brace" for diagnostic in document.diagnostics)
    assert any(record.code == "unclosed-define-brace" and record.text == "*end" for record in document.recovered_records)
    assert document.problems[0].ended


def test_define_integral_header_is_typed_and_scope_checked() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*define integral qmin=-0.10\n"
        "qmin = 0.0;\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    block = next(block for block in document.problems[0].trajectories[0].blocks if block.keyword == "define")
    assert block.integral is True
    assert block.variable == "qmin"
    assert block.initial_value is not None
    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.severity == "error"]

    invalid = parse_problem_text("(demo)\n*define integral qmin=-0.10\nqmin = 0.0;\n*end\n")
    assert any(diagnostic.code == "invalid-integral-define-scope" for diagnostic in invalid.diagnostics)


def test_incomplete_define_statement_is_diagnosed_at_end_of_file() -> None:
    document = parse_problem_text("(demo)\n*define value\nvalue = alt + 1\n*end\n")

    assert any(diagnostic.code == "incomplete-define-statement" for diagnostic in document.diagnostics)
    assert any(record.code == "incomplete-define-statement" for record in document.recovered_records)


def test_undocumented_header_only_block_body_is_preserved_and_diagnosed() -> None:
    document = parse_problem_text("(demo)\n*earth wgs-84\nunexpected text\n*end\n")

    assert any(diagnostic.code == "invalid-block-body" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-block-body" and record.text == "unexpected text" for record in document.recovered_records)


def test_recovered_problem_text_preserves_indentation_and_comments() -> None:
    document = parse_problem_text("(demo)\n*earth wgs-84\n  unexpected text # retain this source\n*end\n")

    record = next(record for record in document.recovered_records if record.code == "invalid-block-body")
    assert record.text == "  unexpected text # retain this source"
    raw_statement = document.problems[0].blocks[0].statements[0]
    assert raw_statement.text == "  unexpected text # retain this source"


def test_diagnostics_are_returned_in_source_order_after_semantic_validation() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 t start on 1\n"
        "*initial geodetic\n"
        "  alt=0\n"
        "*segment 1\n"
        "  *when x&& stop\n"
        "  *aero ca=1 cd=2\n"
        "*segment 1\n"
        "*end\n"
    )

    lines = [diagnostic.location.line for diagnostic in document.diagnostics if diagnostic.location is not None]
    assert lines == sorted(lines)
    assert lines.count(8) == 2


def test_problem_block_preserves_original_header_line() -> None:
    document = parse_problem_text("(demo)\n  *earth wgs-84   # retain header formatting\n*end\n")

    block = document.problems[0].blocks[0]
    assert block.source_text == "  *earth wgs-84   # retain header formatting"
    assert block.header == "wgs-84"
    assert not document.diagnostics


def test_problem_framing_nodes_preserve_original_header_lines() -> None:
    document = parse_problem_text(
        "  (demo)   # problem header\n"
        "  *trajectory 1 vehicle start on 1  # trajectory header\n"
        "    *segment 1 nominal  # segment header\n"
    )

    problem = document.problems[0]
    trajectory = problem.trajectories[0]
    segment = trajectory.segments[0]
    assert problem.source_text == "  (demo)   # problem header"
    assert trajectory.source_text == "  *trajectory 1 vehicle start on 1  # trajectory header"
    assert segment.source_text == "    *segment 1 nominal  # segment header"


def test_units_format_cannot_change_units_of_a_declared_user_variable() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define user-distance\n"
        "user-distance=10;\n"
        "*units/fmt\n"
        "user-distance km f.2\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "units-on-user-defined-variable")
    assert diagnostic.location.line == 5
    record = next(item for item in document.recovered_records if item.code == diagnostic.code)
    assert record.text == "user-distance km f.2"


def test_define_target_cannot_shadow_a_reserved_state_variable() -> None:
    document = parse_problem_text("(demo)\n*define alt\nalt=10;\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "reserved-define-variable")
    assert diagnostic.location.line == 2
    record = next(item for item in document.recovered_records if item.code == diagnostic.code)
    assert record.text == "*define alt"


def test_define_temporary_names_are_reserved_unique_and_noncolliding() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define first\n"
        "alt=1;\n"
        "first=alt;\n"
        "*define second\n"
        "first=2;\n"
        "second=first;\n"
        "*end\n"
    )

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"reserved-define-temporary", "define-variable-collision"} <= codes
    assert {"reserved-define-temporary", "define-variable-collision"} <= {
        record.code for record in document.recovered_records
    }
    assert all(diagnostic.location is not None for diagnostic in document.diagnostics)


def test_duplicate_define_targets_are_preserved_and_diagnosed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define value\n"
        "value=1;\n"
        "*define value\n"
        "value=2;\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "duplicate-define-variable")
    assert diagnostic.location.line == 4
    assert any(record.code == diagnostic.code and record.location.line == 4 for record in document.recovered_records)


def test_duplicate_define_temporary_assignments_are_preserved() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*define value\n"
        "temp=1;\n"
        "temp=2;\n"
        "value=temp;\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "duplicate-define-temporary")
    assert diagnostic.location.line == 4
    assert any(record.code == diagnostic.code and record.location.line == 4 for record in document.recovered_records)


def test_title_continuation_lines_remain_title_text() -> None:
    document = parse_problem_text("(demo)\n*title first line\n  second line\n*end\n")

    assert document.problems[0].blocks[0].title == "first line\nsecond line"


def test_title_preserves_delimiters_but_treats_hash_as_comment_boundary() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*title Flight, Case: A # editorial note\n"
        "*end\n"
    )

    title = document.problems[0].blocks[0]
    assert title.title == "Flight, Case: A"
    assert title.source_text == "*title Flight, Case: A # editorial note"
    assert not document.diagnostics
    assert not document.diagnostics


def test_define_statements_preserve_source_order() -> None:
    document = parse_problem_text("(demo)\n*define x\na = 1; if (alt > 100) then b = 2; else b = 0;\n*end\n")

    block = document.problems[0].blocks[0]
    assert [statement.kind for statement in block.typed_statements] == ["assignment", "if", "else"]
    assert [assignment.name for assignment in block.assignments] == ["a"]


def test_nested_define_control_is_retained() -> None:
    document = parse_problem_text("(demo)\n*define x\nif (alt > 100) then if (vel < 2) then value = 1;\n*end\n")

    control = document.problems[0].blocks[0].control_statements[0]
    assert control.nested is not None
    assert control.nested.assignment is not None


def test_malformed_define_control_gets_a_diagnostic() -> None:
    document = parse_problem_text("(demo)\n*define x\nif alt > ;\n*end\n")

    assert any(diagnostic.code == "invalid-define-control" for diagnostic in document.diagnostics)


def test_recovery_reports_multiple_errors_and_retains_source_records() -> None:
    document = parse_problem_text("(first)\n*unknown\n*trajectory malformed\n*end\n(second)\n*segment 1\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("unknown-block") == 1
    assert "invalid-trajectory-header" in codes
    assert "segment-outside-trajectory" in codes
    assert len(document.recovered_records) >= 3
    assert [problem.name for problem in document.problems] == ["first", "second"]


def test_empty_trajectory_title_is_diagnosed_without_losing_segments() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1  start on 1\n"
        "*segment 1\n"
        "*end\n"
    )

    assert any(diagnostic.code == "invalid-trajectory-header" for diagnostic in document.diagnostics)
    assert document.problems[0].trajectories[0].name == ""
    assert document.problems[0].trajectories[0].segments[0].number == 1
    assert any(record.code == "invalid-trajectory-header" for record in document.recovered_records)


def test_recovery_continues_after_multiple_bad_lines_in_one_file() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory malformed\n"
        "*when altitude > goto 2\n"
        "*initial polar\n"
        "not-an-assignment\n"
        "*unknown\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert codes.count("invalid-trajectory-header") == 1
    assert "invalid-when-condition" in codes
    assert "invalid-initial-header" in codes
    assert "orphan-line" in codes
    assert "unknown-block" in codes
    assert len(document.recovered_records) >= 5


def test_unknown_block_closes_previous_context_without_false_assignment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=1\n"
        "*unknown unsupported\n"
        "  ca=99 # must not become an aero assignment\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    aero = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert aero.assignments[0].name == "ca"
    assert len(aero.assignments) == 1
    assert getattr(aero.assignments[0].value, "value", None) == 1.0
    assert any(item.code == "unknown-block" and item.location.line == 6 for item in document.diagnostics)
    orphan = next(item for item in document.recovered_records if item.code == "orphan-line")
    assert orphan.location.line == 7
    assert orphan.text == "  ca=99 # must not become an aero assignment"
    ####


def test_malformed_framing_headers_close_stale_hierarchy_context() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 good start on 1\n"
        "*segment 1\n"
        "*aero ca=1\n"
        "*trajectory malformed\n"
        "  ca=99 # orphaned after malformed trajectory\n"
        "*segment 2\n"
        "*trajectory 2 second start on 2\n"
        "*segment malformed\n"
        "  ca=88 # orphaned after malformed segment\n"
        "*segment 2 valid\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    problem = document.problems[0]
    assert [trajectory.number for trajectory in problem.trajectories] == [1, 2]
    assert [segment.number for segment in problem.trajectories[0].segments] == [1]
    assert [segment.number for segment in problem.trajectories[1].segments] == [2]
    assert {item.code for item in document.diagnostics} >= {
        "invalid-trajectory-header",
        "segment-outside-trajectory",
        "invalid-segment-header",
    }
    orphan_lines = {item.location.line for item in document.recovered_records if item.code == "orphan-line"}
    assert orphan_lines == {6, 10}
    ####


def test_segment_block_without_segment_is_rejected_without_false_attachment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*aero ca=(table)\n"
        "*end\n"
    )

    assert any(diagnostic.code == "block-outside-segment" for diagnostic in document.diagnostics)
    assert not document.problems[0].blocks
    assert not document.problems[0].trajectories[0].blocks
    assert any(record.code == "block-outside-segment" for record in document.recovered_records)


def test_assignment_body_reports_residual_text_without_rejecting_valid_assignments() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*aero\n"
        "ca=0.1 stray-token\n"
        "*prop\n"
        "thrust=250 another-stray-token\n"
        "*end\n"
    )

    diagnostics = [item for item in document.diagnostics if item.code == "invalid-assignment-line"]
    assert len(diagnostics) == 2
    assert all(item.location.column > 1 for item in diagnostics)
    assert [assignment.name for assignment in document.problems[0].trajectories[0].segments[0].blocks[0].assignments] == ["ca"]
    assert [record.code for record in document.recovered_records].count("invalid-assignment-line") == 2


def test_when_block_has_typed_condition_and_action() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*when time > 10 goto 2\n*end\n")

    block = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert block.keyword == "when"
    assert block.action == "goto"
    assert block.target_segment == 2
    assert block.condition is not None


def test_malformed_when_block_is_diagnosed_and_preserved() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*when time > 10 continue\n*end\n")

    assert any(diagnostic.code == "invalid-when-statement" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-when-statement" and record.text.startswith("*when") for record in document.recovered_records)


def test_initial_block_parses_coordinate_and_copied_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial geodetic\n"
        "alt = 1000\n"
        "wt = 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*end\n"
    )

    first, second = document.problems[0].trajectories[0].blocks[0], document.problems[0].trajectories[0].blocks[1]
    assert first.coordinate_system == "geodetic"
    assert first.assignments[0].name == "alt"
    assert second.source_trajectory == 1
    assert second.source_segment == 1


def test_initial_copy_form_accepts_documented_rotation_assignments() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "  t_0=-1045.2 omega_0=25.3\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    block = document.problems[0].trajectories[0].blocks[0]
    assert [assignment.name for assignment in block.assignments] == ["t_0", "omega_0"]
    assert not document.diagnostics
    ####


def test_initial_copy_form_rejects_undocumented_continuation_assignments_without_guessing() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "  alt=1000\n"
        "*segment 1\n"
        "*end\n"
    )

    assert any(diagnostic.code == "unsupported-initial-copy-parameter" for diagnostic in document.diagnostics)
    assert document.problems[0].trajectories[0].blocks[0].assignments[0].name == "alt"
    assert any(record.code == "unsupported-initial-copy-parameter" and record.text.strip() == "alt=1000" for record in document.recovered_records)
    ####


def test_direct_initial_duplicate_parameter_is_recovered_without_last_write_wins() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial geodetic\n"
        "  alt=1000 wt=1\n"
        "  alt=2000\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "duplicate-initial-parameter")
    assert diagnostic.location.line == 5
    record = next(item for item in document.recovered_records if item.code == "duplicate-initial-parameter")
    assert record.location.line == 5
    assert record.text.strip() == "alt=2000"
    initial = document.problems[0].trajectories[0].blocks[0]
    assert [assignment.name for assignment in initial.assignments] == ["alt", "wt", "alt"]
    ####


def test_limits_continuation_is_typed_but_when_body_is_recovered() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*limits alpha<20\n"
        "  beta>-10 beta<10\n"
        "*when time>1 stop\n"
        "  time=2\n"
        "*end\n"
    )

    limits = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert [(item.variable, item.operator) for item in limits.limits] == [("alpha", "<"), ("beta", ">"), ("beta", "<")]
    records = [record for record in document.recovered_records if record.code == "invalid-block-body"]
    assert len(records) == 1
    assert records[0].text.strip() == "time=2"
    ####


def test_assignment_only_trajectory_blocks_validate_documented_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*dwn/crs latgd=10 long=95 azm=45\n"
        "*iip iip_beta=550 iip_alt=1000\n"
        "*tangent latgd=21.982 long=-159.759 alt=87.2 azm=140\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    blocks = [block for block in document.problems[0].trajectories[0].blocks if block.keyword != "initial"]
    assert [assignment.name for assignment in blocks[0].assignments] == ["latgd", "long", "azm"]
    assert [assignment.name for assignment in blocks[1].assignments] == ["iip_beta", "iip_alt"]
    assert [assignment.name for assignment in blocks[2].assignments] == ["latgd", "long", "alt", "azm"]
    assert not document.diagnostics


def test_assignment_only_block_preserves_malformed_header_and_recovers() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*dwn/crs latgd= long=95 unknown=4 trailing\n"
        "*segment 1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-block-header" in codes
    assert "unsupported-block-parameter" in codes
    assert any(record.code == "invalid-block-header" for record in document.recovered_records)
    assert document.problems[0].trajectories[0].blocks[0].header == "latgd= long=95 unknown=4 trailing"


def test_segment_assignment_blocks_reject_unparsed_header_text() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*aero ca=(drag) stray\n"
        "*constants gravity=32.2\n"
        "*cg cg=0.5\n"
        "*integ dt=0.1 dtprnt=1.0\n"
        "*prop thrust=(thrust) mdot=(mdot)\n"
        "*reset wt=100\n"
        "*increment wt=-1\n"
        "*end\n"
    )

    assert sum(diagnostic.code == "invalid-block-header" for diagnostic in document.diagnostics) == 1
    assert any(record.code == "invalid-block-header" for record in document.recovered_records)
    assert len(document.problems[0].trajectories[0].segments[0].blocks) == 7


def test_limits_block_parses_multiple_relationships() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*limits bankgd>-60 bankgd<60 alpha>-15 alpha<15.0\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    limits = document.problems[0].trajectories[0].segments[0].blocks[0].limits
    assert [(item.variable, item.operator) for item in limits] == [("bankgd", ">"), ("bankgd", "<"), ("alpha", ">"), ("alpha", "<")]
    assert not document.diagnostics


def test_limits_block_recovers_malformed_relationships() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*segment 1\n*limits alpha<\n*end\n")

    assert any(diagnostic.code == "invalid-limits-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-limits-header" for record in document.recovered_records)


def test_fly_block_parses_documented_guidance_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*fly alpha=5.0\n"
        "*fly alpha vrs tseg interp-2\n"
        "*fly l/d-max\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    blocks = document.problems[0].trajectories[0].segments[0].blocks
    assert blocks[0].value is not None
    assert blocks[1].reference == "tseg"
    assert blocks[1].interpolation == "interp-2"
    assert blocks[2].guidance_variable == "l/d-max"
    assert not document.diagnostics


def test_every_manual_guidance_rule_has_a_valid_form() -> None:
    for rule in sorted(SUPPORTED_FLY_GUIDANCE_RULES):
        header = rule if rule == "l/d-max" else f"{rule}=1"
        document = parse_problem_text(
            "(demo)\n"
            "*trajectory 1 vehicle start on 1\n"
            "*initial from trajectory 1, segment 1\n"
            "*segment 1\n"
            f"*fly {header}\n"
            "*when time>1 stop\n"
            "*end\n"
        )

        assert not document.diagnostics, rule
    ####


def test_fly_guidance_table_rows_are_typed_and_source_located() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*fly alpha vrs tseg interp-2\n"
        "  opta-1 0.0\n"
        "  opta-2 opta-3\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    block = document.problems[0].trajectories[0].segments[0].blocks[0]
    assert len(block.points) == 2
    assert block.points[0].location.line == 6
    assert block.points[1].location.line == 7
    assert not document.diagnostics


def test_fly_guidance_table_row_errors_are_recovered() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*fly alpha vrs tseg\n"
        "  0.0\n"
        "  1.0 2.0 3.0\n"
        "*end\n"
    )

    assert [diagnostic.code for diagnostic in document.diagnostics].count("invalid-fly-data") == 2
    assert len([record for record in document.recovered_records if record.code == "invalid-fly-data"]) == 2


def test_fly_rejects_undocumented_variables_and_interpolation_methods() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*fly not-a-guidance-variable=1\n"
        "*fly alpha vrs tseg interp-9\n"
        "0 1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "unsupported-fly-variable" in codes
    assert "unsupported-fly-interpolation" in codes
    assert not any(record.code == "orphan-line" for record in document.recovered_records)


@pytest.mark.parametrize(
    ("first", "second", "third"),
    [
        ("alpha", "betae", "bankgc"),
        ("alpha", "beta", "bankgc"),
        ("alphat", "phi", "bankgc"),
        ("alpha", "betae", "bankgd"),
        ("alpha", "beta", "bankgd"),
        ("alphat", "phi", "bankgd"),
        ("yawgc", "pitchgc", "rollgc"),
        ("yawgd", "pitchgd", "rollgd"),
        ("yawi", "pitchi", "rolli"),
    ],
)
def test_fly_accepts_all_manual_table_4_3_angle_sets(first: str, second: str, third: str) -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        f"*fly {first}=0\n"
        f"*fly {second}=0\n"
        f"*fly {third}=0\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert not [diagnostic for diagnostic in document.diagnostics if diagnostic.code == "inconsistent-fly-angle-set"]
    ####


def test_fly_and_rail_malformed_forms_are_recovered() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*fly alpha vrs\n"
        "*rail catapult cfstat=0.1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-fly-statement" in codes
    assert "invalid-rail-header" in codes
    assert len(document.recovered_records) >= 2


def test_rail_rejects_undocumented_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*rail launch cfstat=0.1 cfslid=0.02 friction=0.3\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    assert any(diagnostic.code == "unsupported-block-parameter" for diagnostic in document.diagnostics)
    assert len(document.recovered_records) == 1


def test_integration_and_reset_increment_vocabularies_are_checked() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*integ dt=0.1 dtprnt=1.0 dtguid=5.0 mystery=2\n"
        "*reset wt=100 mystery=1\n"
        "*increment velibx=5 dxb=0.1\n"
        "*end\n"
    )

    unsupported = [diagnostic for diagnostic in document.diagnostics if diagnostic.code == "unsupported-block-parameter"]
    assert len(unsupported) == 2
    assert not any(diagnostic.code == "unsupported-block-parameter" and "velibx" in diagnostic.message for diagnostic in document.diagnostics)


def test_aero_rejects_mixed_documented_coefficient_sets() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*segment 1\n"
        "*aero ca=0.1 cn=0.2 cl=0.3 flaps=25\n"
        "*end\n"
    )

    assert any(diagnostic.code == "inconsistent-aero-coefficients" for diagnostic in document.diagnostics)
    assert any(record.code == "inconsistent-aero-coefficients" for record in document.recovered_records)


def test_propulsion_unit_controls_use_their_documented_unit_tables() -> None:
    valid = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*prop thrust=250 thr_units=kn mdot=120 mdt_units=kg/sec nrecruits=2\n"
        "*when time>1 stop\n"
        "*end\n"
    )
    assert not [diagnostic for diagnostic in valid.diagnostics if diagnostic.severity.value == "error"]

    invalid = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*prop thrust=250 thr_units=psi mdot=120 mdt_units=kg\n"
        "*when time>1 stop\n"
        "*end\n"
    )
    assert [diagnostic.code for diagnostic in invalid.diagnostics].count("unsupported-propulsion-unit") == 2


def test_optimize_constraints_and_controls_are_typed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 6, trajectory 1\n"
        "constrain vel=3000\n"
        "constrain alt on segment 3, trajectory 2 = 3000\n"
        "constrain east[2] on segment 4 = east[1] on segment 7, ref=1000\n"
        "constrain gamgd on segment 6, trajectory 1\n"
        " = gamgd on segment 4, trajectory 3\n"
        "fref=100 maxitr=40 tol=1.0e-7\n"
        "par-1=9.9 lo-1=-15 hi-1=15\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert len(block.constraints) == 4
    assert block.constraints[0].left.text == "vel"
    assert block.constraints[1].left.segment == 3
    assert block.constraints[2].right.trajectory_subscript == 1
    assert block.constraints[2].reference is not None
    assert [assignment.name for assignment in block.controls] == ["fref", "maxitr", "tol", "par-1", "lo-1", "hi-1"]
    assert not document.diagnostics


def test_optimize_accepts_every_documented_control_variable() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize e for range=min on segment 12, trajectory 3\n"
        "constrain vel=3000\n"
        "fref=100 derivs=0 tol=1.0e-7 dx=1.0e-8 maxitr=40\n"
        "adjust=1 integ=0 surveys=1 restarts=2 print=1\n"
        "par-1=9.9 lo-1=-15 hi-1=15 ref-1=10\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert isinstance(block, OptimizeBlock)
    assert block.loop == "e"
    assert block.objective_mode == "min"
    assert {assignment.name for assignment in block.controls} == {
        "fref", "derivs", "tol", "dx", "maxitr", "adjust", "integ", "surveys", "restarts", "print",
        "par-1", "lo-1", "hi-1", "ref-1",
    }
    assert not document.diagnostics


def test_search_and_optimize_controls_require_numeric_values() -> None:
    search = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until alt=30000 on segment 1, trajectory 1\n"
        "xlo=low xhi=10 xest=5 dx=1\n"
        "*end\n"
    )
    assert any(item.code == "nonnumeric-search-control" for item in search.diagnostics)
    assert any(record.code == "nonnumeric-search-control" for record in search.recovered_records)

    optimize = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "fref=scale par-1=guess\n"
        "*end\n"
    )
    assert [item.code for item in optimize.diagnostics].count("nonnumeric-optimize-control") == 2
    assert all(record.code == "nonnumeric-optimize-control" for record in optimize.recovered_records)


def test_search_and_optimize_reject_unparsed_body_lines_without_assigning_meaning() -> None:
    search = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until alt=30000 on segment 1, trajectory 1\n"
        "not a control\n"
        "*end\n"
    )
    optimize = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "not a control\n"
        "*end\n"
    )

    assert any(item.code == "invalid-search-control-line" for item in search.diagnostics)
    assert any(item.code == "invalid-optimize-control-line" for item in optimize.diagnostics)
    assert any(item.code == "invalid-search-control-line" for item in search.recovered_records)
    assert any(item.code == "invalid-optimize-control-line" for item in optimize.recovered_records)
    assert not search.problems[0].blocks[0].controls
    assert not optimize.problems[0].blocks[0].controls


def test_optimize_constraint_recovery_continues_to_controls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain missing endpoint\n"
        "fref=100 maxitr=2\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert any(diagnostic.code == "invalid-optimize-constraint" for diagnostic in document.diagnostics)
    assert [assignment.name for assignment in block.controls] == ["fref", "maxitr"]
    assert any(record.code == "invalid-optimize-constraint" for record in document.recovered_records)


def test_multiline_optimize_constraint_is_typed_once() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain vel on segment 1\n"
        "  = 3000\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert not document.diagnostics
    assert len(block.constraints) == 1
    assert block.constraints[0].left.text == "vel"
    assert block.constraints[0].right.text == "3000"
    ####


def test_optimize_header_and_control_diagnostics_preserve_source() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize malformed\n"
        "foo=1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-optimize-header" in codes
    assert "unsupported-optimize-control" in codes
    assert document.problems[0].blocks[0].header == "malformed"
    assert document.problems[0].blocks[0].controls[0].name == "foo"


def test_optimize_rejects_undocumented_wildcard_endpoint() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain * = 0\n"
        "*end\n"
    )

    assert any(diagnostic.code == "invalid-optimize-endpoint" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-optimize-endpoint" for record in document.recovered_records)


def test_inertial_block_parses_documented_alignment_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*inertial platform alignment geocentric\n"
        "*inertial platform lat=28.8 long=-81.5 alt=550 time=100\n"
        "*inertial ecfc eastx=0.7071 easty=0.7071 eastz=0.0 downz=-1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*end\n"
    )

    blocks = [block for block in document.problems[0].trajectories[0].blocks if block.keyword == "inertial"]
    assert blocks[0].alignment == "geocentric"
    assert blocks[0].coordinate_system == "geocentric"
    assert blocks[1].alignment == "geodetic"
    assert blocks[2].alignment == "ecfc"
    assert not document.diagnostics


def test_inertial_block_rejects_ambiguous_alignment_text() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*inertial polar\n*segment 1\n*end\n")

    assert any(diagnostic.code == "invalid-inertial-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-inertial-header" for record in document.recovered_records)


def test_inertial_body_alignment_is_rejected_on_first_segment() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial ecfc x=1 y=0 z=0 xdt=0 ydt=0 zdt=0 time=0 mass=1\n"
        "*segment 1\n"
        "*inertial body\n"
        "*when time=1 stop\n"
        "*end\n"
    )

    diagnostic = next(item for item in document.diagnostics if item.code == "inertial-body-first-segment")
    assert diagnostic.location.line == 5
    assert any(record.code == diagnostic.code and record.location.line == 5 for record in document.recovered_records)


def test_survey_block_parses_incremental_and_explicit_values() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 1 alt0\n"
        "lo=40000 hi=80000 inc=20000\n"
        "vals=35000,45000\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.survey_id == 1
    assert block.name == "alt0"
    assert [(setting.name, setting.values) for setting in block.settings] == [
        ("lo", ["40000"]),
        ("hi", ["80000"]),
        ("inc", ["20000"]),
        ("vals", ["35000", "45000"]),
    ]
    assert not document.diagnostics


def test_survey_block_recovers_bad_header_and_setting() -> None:
    document = parse_problem_text("(demo)\n*survey missing\nlo=one unknown=2\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-survey-header" in codes
    assert "invalid-survey-value" in codes
    assert "invalid-survey-setting" in codes
    assert len(document.recovered_records) >= 2


def test_survey_continuation_recovers_each_diagnostic_source_line() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 1 alt0\n"
        "lo=one unknown=2\n"
        "*end\n"
    )

    codes = {record.code for record in document.recovered_records}
    assert {"invalid-survey-value", "invalid-survey-setting"} <= codes
    continuation_records = [
        record
        for record in document.recovered_records
        if record.code in {"invalid-survey-value", "invalid-survey-setting"}
    ]
    assert all(record.text.strip() == "lo=one unknown=2" for record in continuation_records)
    ####


def test_survey_requires_complete_incremental_or_explicit_values() -> None:
    incomplete = parse_problem_text("(demo)\n*survey 1 alt0 lo=1 hi=3\n*end\n")
    assert any(diagnostic.code == "incomplete-survey-increment" for diagnostic in incomplete.diagnostics)
    assert any(record.code == "incomplete-survey-increment" for record in incomplete.recovered_records)

    empty = parse_problem_text("(demo)\n*survey 1 alt0\n*end\n")
    assert any(diagnostic.code == "missing-survey-values" for diagnostic in empty.diagnostics)
    assert any(record.code == "missing-survey-values" for record in empty.recovered_records)

    explicit = parse_problem_text("(demo)\n*survey 1 alt0 vals=1,2\n*end\n")
    assert not explicit.diagnostics
    ####


def test_duplicate_incremental_survey_setting_is_ambiguous_but_preserved() -> None:
    document = parse_problem_text("(demo)\n*survey 1 alt0 lo=1 lo=2 hi=3 inc=1\n*end\n")

    block = document.problems[0].blocks[0]
    assert [setting.name for setting in block.settings] == ["lo", "lo", "hi", "inc"]
    assert any(diagnostic.code == "duplicate-survey-setting" for diagnostic in document.diagnostics)
    assert any(record.code == "duplicate-survey-setting" for record in document.recovered_records)
    ####


def test_duplicate_explicit_survey_values_are_ambiguous_but_preserved() -> None:
    document = parse_problem_text("(demo)\n*survey 1 alt0 vals=1,2 vals=3,4\n*end\n")

    block = document.problems[0].blocks[0]
    assert [setting.values for setting in block.settings] == [["1", "2"], ["3", "4"]]
    assert any(diagnostic.code == "duplicate-survey-setting" for diagnostic in document.diagnostics)
    assert any(record.code == "duplicate-survey-setting" for record in document.recovered_records)
    ####


def test_duplicate_survey_ids_do_not_resolve_placeholders() -> None:
    result = ingest_text(
        """(survey-check)
*define result
  result=surv-1;
*survey 1 alt0 vals=1
*survey 1 alt1 vals=2
*end
""",
        kind=FileKind.PROBLEM,
    )

    codes = {diagnostic.code for diagnostic in result.diagnostics}
    assert {"duplicate-survey-id", "unknown-survey-reference"} <= codes
    ####


def test_survey_and_search_numbers_must_begin_at_one() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 0 alt vals=1\n"
        "*search 0 vary alpha until min\n"
        "*end\n"
    )

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"invalid-survey-id", "invalid-search-id"} <= codes
    assert {"invalid-survey-id", "invalid-search-id"} <= {record.code for record in document.recovered_records}
    ####


def test_survey_name_cannot_duplicate_output_variable_name() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*survey 1 alt vals=1\n"
        "*print alt\n"
        "*end\n"
    )

    assert any(diagnostic.code == "conflicting-survey-output-name" for diagnostic in document.diagnostics)
    assert any(record.code == "conflicting-survey-output-name" for record in document.recovered_records)


def test_define_requires_assignment_to_declared_variable() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*define result\n"
        "temporary=1;\n"
        "*end\n"
    )

    assert any(diagnostic.code == "missing-define-target" for diagnostic in document.diagnostics)
    assert any(record.code == "missing-define-target" for record in document.recovered_records)


def test_define_after_segment_attaches_at_trajectory_scope() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*initial from trajectory 1, segment 1\n"
        "*segment 1\n"
        "*when time>1 stop\n"
        "*define result\n"
        "result=alt;\n"
        "*end\n"
    )

    trajectory = document.problems[0].trajectories[0]
    assert [block.keyword for block in trajectory.blocks if block.keyword != "initial"] == ["define"]
    assert [block.keyword for block in trajectory.segments[0].blocks] == ["when"]
    assert [item.code for item in document.diagnostics] == ["ambiguous-dual-scope-block"]
    assert document.diagnostics[0].severity.value == "warning"
    assert document.recovered_records[0].code == "ambiguous-dual-scope-block"


def test_search_block_parses_objective_continuation_and_controls() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary pitch-rate until pitchi on segment 2\n"
        " = pitchi on segment 3, trajectory 2\n"
        "xlo=0.5 xhi=10.0 xest=5.0 dx=1.0 tol=0.001\n"
        "xref=1.0 fref=10000 maxitr=20 print=1 integ=0\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.search_id == 1
    assert block.variable == "pitch-rate"
    assert block.objective is not None
    assert block.objective.operator == "="
    assert block.objective.left.segment == 2
    assert block.objective.left.trajectory == 2
    assert block.objective.right.segment == 3
    assert block.objective.right.trajectory == 2
    assert [assignment.name for assignment in block.controls] == [
        "xlo", "xhi", "xest", "dx", "tol", "xref", "fref", "maxitr", "print", "integ"
    ]
    assert not document.diagnostics


def test_search_block_recovers_incomplete_objective_and_unknown_control() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until alt on segment 3\n"
        "unknown=1\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-search-objective" in codes
    assert "unsupported-search-control" in codes
    assert any(record.code == "invalid-search-objective" for record in document.recovered_records)


def test_duplicate_search_controls_are_diagnosed_without_overwriting_source() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until alt=0 on segment 1, trajectory 1\n"
        "xlo=0 xhi=1 xest=0.5 dx=0.1 xlo=-1\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert [assignment.name for assignment in block.controls] == ["xlo", "xhi", "xest", "dx", "xlo"]
    assert any(diagnostic.code == "duplicate-search-control" for diagnostic in document.diagnostics)
    ####


def test_duplicate_search_ids_do_not_resolve_placeholders() -> None:
    result = ingest_text(
        """(search-check)
*define result
  result=srch-1;
*search 1 vary alpha until alt=0 on segment 1, trajectory 1
  xlo=0 xhi=1 xest=0.5 dx=0.1
*search 1 vary beta until alt=0 on segment 1, trajectory 1
  xlo=0 xhi=1 xest=0.5 dx=0.1
*end
""",
        kind=FileKind.PROBLEM,
    )

    codes = {diagnostic.code for diagnostic in result.diagnostics}
    assert {"duplicate-search-id", "unknown-search-reference"} <= codes
    ####


def test_search_accepts_documented_min_and_max_objectives() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until min\n"
        "*search 2 vary beta until max\n"
        "*end\n"
    )

    assert not document.diagnostics
    assert [block.objective.left.text for block in document.problems[0].blocks] == ["min", "max"]


def test_search_rejects_numeric_first_objective_term() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*search 1 vary alpha until 30000 on segment 1, trajectory 1\n"
        "*end\n"
    )

    assert any(diagnostic.code == "invalid-search-endpoint" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-search-endpoint" for record in document.recovered_records)


def test_search_rejects_missing_first_endpoint_location() -> None:
    document = parse_problem_text("(demo)\n*search 1 vary alpha until alt\n*end\n")

    assert any(diagnostic.code == "invalid-search-endpoint" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-search-endpoint" for record in document.recovered_records)
    ####


def test_endpoint_rejects_conflicting_trajectory_qualifiers() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*optimize a for vel=max on segment 1, trajectory 1\n"
        "constrain alt[2] on segment 3, trajectory 2 = 3000\n"
        "*end\n"
    )

    assert any(diagnostic.code == "conflicting-endpoint-trajectory" for diagnostic in document.diagnostics)
    assert any(record.code == "conflicting-endpoint-trajectory" for record in document.recovered_records)
    ####


def test_optimize_header_requires_trajectory_number() -> None:
    document = parse_problem_text("(demo)\n*optimize a for vel=max on segment 1\n*end\n")

    assert any(diagnostic.code == "invalid-optimize-header" for diagnostic in document.diagnostics)
    ####


def test_radar_block_parses_station_identity_shape_and_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*radar 2 station_y alt=112.4 long=65.345 latgd=12.435 wgs-72 distn=25.5 diste=15.2 distd=-10.3\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.radar_id == 2
    assert block.station_name == "station_y"
    assert block.earth_shape == "wgs-72"
    assert [assignment.name for assignment in block.assignments] == ["alt", "long", "latgd", "distn", "diste", "distd"]
    assert not document.diagnostics


def test_radar_continuation_preserves_shape_and_typed_parameters() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*radar 2 station_y alt=112.4 long=65.345\n"
        "  wgs-72 distn=25.5 diste=15.2 distd=-10.3\n"
        "*end\n"
    )

    block = document.problems[0].blocks[0]
    assert block.earth_shape == "wgs-72"
    assert [assignment.name for assignment in block.assignments] == ["alt", "long", "distn", "diste", "distd"]
    assert not document.diagnostics


def test_radar_direct_earth_shape_requires_radius_and_one_polar_parameter() -> None:
    valid = parse_problem_text("(demo)\n*radar 1 station reqtr=1 rpolr=2\n*end\n")
    assert not valid.diagnostics

    invalid = parse_problem_text(
        "(demo)\n"
        "*radar 1 station rpolr=2 ecc=0.1\n"
        "*end\n"
    )
    codes = {diagnostic.code for diagnostic in invalid.diagnostics}
    assert {"missing-radar-equatorial-radius", "conflicting-radar-shape-parameters"} <= codes
    assert all(
        any(record.code == diagnostic.code and record.location.line == diagnostic.location.line for record in invalid.recovered_records)
        for diagnostic in invalid.diagnostics
    )


def test_radar_named_earth_shape_cannot_be_mixed_with_direct_geometry() -> None:
    document = parse_problem_text("(demo)\n*radar 1 station reqtr=1 wgs-84\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "conflicting-radar-earth-shape-definition")
    assert diagnostic.location.line == 2
    assert any(record.code == diagnostic.code for record in document.recovered_records)


def test_fixed_vocabulary_continuation_rejects_unknown_parameters_with_recovery() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*trajectory 1 vehicle start on 1\n"
        "*dwn/crs\n"
        "  latgd=10 mystery=4 another=9\n"
        "*segment 1\n"
        "*inertial geocentric\n"
        "  long=95 mystery=4\n"
        "*end\n"
    )

    unsupported = [diagnostic for diagnostic in document.diagnostics if diagnostic.code == "unsupported-block-parameter"]
    assert len(unsupported) == 3
    assert len({diagnostic.location.column for diagnostic in unsupported}) == 3
    assert {record.location.line for record in document.recovered_records if record.code == "unsupported-block-parameter"} == {4, 7}
    ####


def test_radar_block_recovers_missing_identity_and_unknown_parameter() -> None:
    document = parse_problem_text("(demo)\n*radar missing alt=1 unknown=2\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-radar-header" in codes
    assert any(record.code == "invalid-radar-header" for record in document.recovered_records)


def test_radar_station_numbers_are_positive_and_unique() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*radar 0 ground alt=1\n"
        "*radar 1 first alt=2\n"
        "*radar 1 second alt=3\n"
        "*end\n"
    )

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"invalid-radar-id", "duplicate-radar-id"} <= codes
    assert {"invalid-radar-id", "duplicate-radar-id"} <= {record.code for record in document.recovered_records}
    assert [block.station_name for block in document.problems[0].blocks] == ["ground", "first", "second"]
    ####


def test_duplicate_radar_parameters_are_preserved_and_diagnosed() -> None:
    document = parse_problem_text("(demo)\n*radar 1 station alt=1 alt=2\n*end\n")

    block = document.problems[0].blocks[0]
    assert [assignment.value.value for assignment in block.assignments] == [1.0, 2.0]
    assert any(diagnostic.code == "duplicate-radar-parameter" for diagnostic in document.diagnostics)
    assert any(record.code == "duplicate-radar-parameter" for record in document.recovered_records)
    ####


def test_units_format_block_parses_inline_and_continuation_settings() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*units/fmt sref in\n"
        "alt km f.3\n"
        "vel m/sec f.2\n"
        "range km\n"
        "mach e.5\n"
        "*end\n"
    )

    settings = document.problems[0].blocks[0].settings
    assert [(setting.variable, setting.unit, setting.format) for setting in settings] == [
        ("sref", "in", None),
        ("alt", "km", "f.3"),
        ("vel", "m/sec", "f.2"),
        ("range", "km", None),
        ("mach", None, "e.5"),
    ]
    assert not document.diagnostics


def test_units_format_block_recovers_malformed_setting() -> None:
    document = parse_problem_text("(demo)\n*units/fmt alt km f.x\nlonely\n*end\n")

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-units-format" in codes
    assert "invalid-units-format-setting" in codes
    assert len(document.recovered_records) >= 2


def test_units_format_rejects_units_outside_the_manual_table() -> None:
    document = parse_problem_text("(demo)\n*units/fmt alt parsec\n*end\n")

    assert any(diagnostic.code == "unsupported-unit" for diagnostic in document.diagnostics)


def test_units_format_rejects_dimensionally_incompatible_units() -> None:
    document = parse_problem_text("(demo)\n*units/fmt alt sec\n*end\n")

    diagnostic = next(item for item in document.diagnostics if item.code == "incompatible-unit-dimension")
    assert diagnostic.location.line == 2
    assert any(record.code == diagnostic.code for record in document.recovered_records)


def test_units_format_rejects_malformed_variable_names_with_recovery() -> None:
    document = parse_problem_text("(demo)\n*units/fmt bad$name km f.2\n*end\n")

    assert any(diagnostic.code == "invalid-units-format-variable" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-units-format-variable" for record in document.recovered_records)
    ####


def test_atmosphere_and_earth_headers_and_user_rows_are_typed() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd visc\n"
        "0 548.2 2111.0 0.00224 1147 3.895e-7\n"
        "*earth wgs-84 omega=0\n"
        "*end\n"
    )

    atmos, earth = document.problems[0].blocks
    assert atmos.columns == ["alt", "temp", "pres", "rho", "sndspd", "visc"]
    assert atmos.rows[0][0] == 0.0
    assert earth.model == "wgs-84"
    assert earth.assignments[0].name == "omega"
    assert not document.diagnostics


def test_user_atmosphere_accepts_documented_kinematic_viscosity_column() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos user\n"
        "alt temp pres rho sndspd nu\n"
        "0 518.67 2116.22 0.0023769 1116.45 0.000157\n"
        "*end\n"
    )

    atmos = document.problems[0].blocks[0]
    assert atmos.columns[-1] == "nu"
    assert not document.diagnostics


def test_atmosphere_and_earth_recover_bad_forms() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos 21\n"
        "*atmos site\n"
        "alt pressure rho\n"
        "*atmos site\n"
        "alt pres rho\n"
        "0 bad 0.002\n"
        "*earth unknown omega=0\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "invalid-atmos-header" in codes
    assert "invalid-atmos-columns" in codes
    assert "invalid-atmos-row" in codes
    assert "invalid-earth-header" in codes
    assert len(document.recovered_records) >= 4


def test_atmosphere_altitudes_must_increase_and_earth_shape_is_unambiguous() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*atmos site\n"
        "alt pres rho\n"
        "10000 1485 0.0016\n"
        "10000 1480 0.0015\n"
        "0 2115 0.0022\n"
        "*earth wgs-84 reqtr=1 rpolr=2 flat=3\n"
        "*end\n"
    )

    codes = [diagnostic.code for diagnostic in document.diagnostics]
    assert "duplicate-atmos-altitude" in codes
    assert "unordered-atmos-altitude" in codes
    assert "conflicting-earth-shape-parameters" in codes
    assert any(record.code == "duplicate-atmos-altitude" for record in document.recovered_records)
    assert any(record.code == "conflicting-earth-shape-parameters" for record in document.recovered_records)
    assert next(item for item in document.diagnostics if item.code == "duplicate-atmos-altitude").location.line == 5
    ####


def test_duplicate_global_models_and_earth_parameters_are_preserved() -> None:
    document = parse_problem_text(
        "(demo)\n"
        "*earth wgs-84 omega=0 omega=1\n"
        "*earth wgs-72\n"
        "*atmos standard\n"
        "*atmos none\n"
        "*end\n"
    )

    codes = {diagnostic.code for diagnostic in document.diagnostics}
    assert {"duplicate-earth-model", "duplicate-atmosphere-model", "duplicate-earth-parameter"} <= codes
    assert {"duplicate-earth-model", "duplicate-atmosphere-model", "duplicate-earth-parameter"} <= {record.code for record in document.recovered_records}
    assert [assignment.value.value for assignment in document.problems[0].blocks[0].assignments] == [0.0, 1.0]
    ####


def test_invalid_initial_form_is_diagnosed_and_recovered() -> None:
    document = parse_problem_text("(demo)\n*trajectory 1 vehicle start on 1\n*initial polar\n*segment 1\n*end\n")

    assert any(diagnostic.code == "invalid-initial-header" for diagnostic in document.diagnostics)
    assert any(record.code == "invalid-initial-header" for record in document.recovered_records)


def test_else_if_chain_is_nested_as_typed_control() -> None:
    document = parse_problem_text("(demo)\n*define x\nif (alt > 100) then value = 1;\nelse if (alt > 50) then value = 2;\nelse value = 0;\n*end\n")

    controls = document.problems[0].blocks[0].control_statements
    assert controls[1].kind == "else"
    assert controls[1].nested is not None
    assert controls[1].nested.kind == "if"
