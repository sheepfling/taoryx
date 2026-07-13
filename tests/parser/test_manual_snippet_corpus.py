from __future__ import annotations

import json
from pathlib import Path

from taoryx.language._legacy_table import TABLE_OPERATIONS, TABLE_TYPES
from taoryx.language.grammar_contracts import (
    SUPPORTED_PROBLEM_BLOCKS,
    SUPPORTED_SEGMENT_BLOCKS,
    SUPPORTED_TRAJECTORY_BLOCKS,
)
from taoryx.language.ingest import FileKind, ingest_text
from taoryx.language.lexical import lex_text
from taoryx.language.lossless import parse_lossless_bytes
from taoryx.language.problem_fragments import parse_optimize_body_fragment, parse_problem_fragment
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import (
    parse_simple_table_body_fragment,
    parse_skewed_assignment_groups_fragment,
    parse_table_assignment_fragment,
    parse_table_body_fragment,
    parse_table_header_fragment,
    parse_table_operation_fragment,
    parse_table_text,
)

CORPUS = Path(__file__).parents[1] / "fixtures" / "taos_manual_corpus_v22"
MANIFEST = json.loads((CORPUS / "manifest.json").read_text(encoding="utf-8"))
EXPECTED_WRAPPER_ERRORS = {"ch3-011": 1, "ch3-022": 1, "ch4-005": 1, "ch4-016": 1, "ch4-017": 3, "ch4-026": 3, "ch4-029": 1, "ch4-051": 1, "ch4-080": 2}
EXPECTED_WRAPPER_DIAGNOSTIC_CODES = {
    "ch3-011": {"independent-assignment-order"},
    "ch3-022": {"missing-interpolation-data"},
    "ch4-005": {"invalid-assignment-line"},
    "ch4-016": {"inconsistent-fly-angle-set"},
    "ch4-017": {"wildcard-fly-in-first-segment"},
    "ch4-026": {"wildcard-fly-in-first-segment"},
    "ch4-029": {"inertial-body-first-segment"},
    "ch4-051": {"missing-trajectory-initial"},
    "ch4-080": {"missing-egs-summary-survey", "missing-egs-summary-variable"},
}


def test_corpus_manifest_and_every_raw_display_are_lossless() -> None:
    entries = MANIFEST["entries"]
    assert len(entries) == 164
    summary = MANIFEST["summary"]
    assert summary["total_displays"] == len(entries)
    assert summary["phase1_parser_attempted"] == sum(bool(entry["wrapper_path"]) for entry in entries)
    assert summary["phase1_parser_exceptions"] == 0
    assert summary["source_monospace_uncaptured"] == 0
    assert len({entry["id"] for entry in entries}) == 164
    for entry in entries:
        raw = (CORPUS / entry["raw_path"]).read_bytes()
        assert parse_lossless_bytes(raw, source_path=entry["raw_path"]).render_bytes() == raw
        if entry["wrapper_path"]:
            assert (CORPUS / entry["wrapper_path"]).is_file()
    ####


def test_source_final_pass_accounting_matches_the_reviewed_audit() -> None:
    source_pass = json.loads((CORPUS / "source_context" / "SOURCE_FINAL_PASS.json").read_text(encoding="utf-8"))
    audit = json.loads((CORPUS / "source_context" / "source_monospace_audit.json").read_text(encoding="utf-8"))

    assert len(audit) == source_pass["source_monospace_blocks_reviewed"] == 455
    assert {item["review_status"] for item in audit} == {"automatic_match", "manual_match", "not_a_snippet"}
    assert source_pass["uncaptured_blocks"] == 0
    ####


def test_complete_top_level_corpus_documents_have_no_parser_errors() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["parse_mode"] in {"top_level", "top_level_excerpt"} and entry["language"] in {"tbl", "prb"}]
    assert len(entries) == 17
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        parsed = parse_table_text(path.read_text(), str(path)) if entry["language"] == "tbl" else parse_problem_text(path.read_text(), str(path))
        assert not [diagnostic for diagnostic in parsed.diagnostics if diagnostic.severity.value == "error"], entry["id"]
    ####


def test_declared_operation_fragment_uses_fragment_parser_without_false_semantics() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch3-022")
    path = CORPUS / entry["raw_path"]
    text = path.read_text(encoding="utf-8")

    fragment = parse_table_operation_fragment(text, str(path))

    assert fragment.source_text == text
    assert [operation.operator for operation in fragment.operations] == ["add", "cos", "sqr", "csto", "add"]
    assert fragment.operations[-1].operand.name == "factor"
    assert fragment.operations[-1].operand.arguments == ["ca2", "mach"]
    assert not fragment.diagnostics
    assert not fragment.recovered_records
    ####


def test_declared_table_body_fragments_use_the_body_parser() -> None:
    entries = [item for item in MANIFEST["entries"] if item["recommended_entrypoint"] == "parse_full_table_body"]
    assert {item["id"] for item in entries} == {"ch3-016", "ch3-028"}
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        fragment = parse_table_body_fragment(path.read_text(encoding="utf-8"), str(path))
        assert not fragment.diagnostics, entry["id"]
        assert fragment.operations
        assert fragment.source_text == path.read_text(encoding="utf-8")
    ####


def test_declared_assignment_fragments_use_the_assignment_parser() -> None:
    entries = [item for item in MANIFEST["entries"] if item["recommended_entrypoint"] == "parse_table_assignment"]
    assert {item["id"] for item in entries} == {"ch3-008", "ch3-009"}
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        fragment = parse_table_assignment_fragment(path.read_text(encoding="utf-8"), str(path))
        assert not fragment.diagnostics, entry["id"]
        assert fragment.assignments
        assert fragment.source_text == path.read_text(encoding="utf-8")
    ####


def test_declared_header_fragments_use_the_header_parser() -> None:
    expected = {
        "ch3-005": ("ca", ["mach", "alt", "alpha"], {}),
        "ch3-006": ("ca", ["mach", "alpha"], {"extrapolation": "extrap", "sref": 2.162}),
        "ch3-007": ("mdot", ["time", "motor"], {"extrapolation": "no-extrap", "units": "lb/hr"}),
    }
    for entry_id, (table_type, variables, options) in expected.items():
        entry = next(item for item in MANIFEST["entries"] if item["id"] == entry_id)
        path = CORPUS / entry["raw_path"]
        fragment = parse_table_header_fragment(path.read_text(encoding="utf-8"), str(path))
        assert fragment.table_type == table_type
        assert fragment.independent_variables == variables
        assert fragment.options == options
        assert not fragment.diagnostics, entry_id
        assert fragment.source_text == path.read_text(encoding="utf-8")
    ####


def test_declared_simple_table_body_fragments_preserve_incomplete_examples() -> None:
    entries = [item for item in MANIFEST["entries"] if item["recommended_entrypoint"] == "parse_simple_table_body"]
    assert {item["id"] for item in entries} == {"ch3-010", "ch3-011"}
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        fragment = parse_simple_table_body_fragment(path.read_text(encoding="utf-8"), str(path))
        assert fragment.header.table_type == "cx"
        assert fragment.header.independent_variables == ["alt", "mach"]
        assert [assignment.name for assignment in fragment.assignments] == ["alt", "mach"] if entry["id"] == "ch3-010" else ["mach", "alt"]
        assert not fragment.diagnostics, entry["id"]
        assert fragment.source_text == path.read_text(encoding="utf-8")
    ####


def test_declared_skewed_fragments_preserve_assignment_group_boundaries() -> None:
    expected_groups = {"ch3-029": 2, "ch3-030": 4, "ch3-031": 5}
    entries = [item for item in MANIFEST["entries"] if item["recommended_entrypoint"] == "parse_skewed_assignment_groups"]
    assert {item["id"] for item in entries} == set(expected_groups)
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        fragment = parse_skewed_assignment_groups_fragment(path.read_text(encoding="utf-8"), str(path))
        assert fragment.operation is not None, entry["id"]
        assert fragment.operation.operator == "add"
        assert len(fragment.assignment_groups) == expected_groups[entry["id"]]
        assert all(group for group in fragment.assignment_groups)
        assert not fragment.diagnostics, entry["id"]
        assert fragment.source_text == path.read_text(encoding="utf-8")
    ####


def test_direct_problem_fragments_use_the_declared_scope_without_context_errors() -> None:
    scopes = {
        "parse_problem_block": "problem",
        "parse_trajectory_block": "trajectory",
        "parse_segment_block": "segment",
    }
    entries = []
    for entry in MANIFEST["entries"]:
        if entry["parse_mode"] != "fragment" or entry["language"] != "prb":
            continue
        if entry["recommended_entrypoint"] not in scopes:
            continue
        path = CORPUS / entry["raw_path"]
        first = next((line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()), "")
        if first.lstrip().startswith("*"):
            entries.append((entry, path, scopes[entry["recommended_entrypoint"]]))
    assert len(entries) == 102
    for entry, path, scope in entries:
        fragment = parse_problem_fragment(path.read_text(encoding="utf-8"), scope=scope, path=str(path))
        assert not fragment.diagnostics, entry["id"]
        assert fragment.source_text == path.read_text(encoding="utf-8")
        assert all(record.location.line > 0 for record in fragment.recovered_records), entry["id"]
        assert all(record.location.line > 0 for record in fragment.deferred_recovered_records), entry["id"]
    ####


def test_scoped_fragment_defers_contextual_semantics_without_discarding_evidence() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-017")
    path = CORPUS / entry["raw_path"]
    fragment = parse_problem_fragment(path.read_text(encoding="utf-8"), scope="segment", path=str(path))

    assert not fragment.diagnostics
    assert {item.code for item in fragment.deferred_diagnostics} >= {"wildcard-fly-in-first-segment"}
    assert {item.code for item in fragment.deferred_recovered_records} >= {"wildcard-fly-in-first-segment"}
    assert all(item.location.line > 0 for item in fragment.deferred_diagnostics)
    ####


def test_optimize_body_fragments_preserve_constraints_and_controls_without_header_semantics() -> None:
    entries = [item for item in MANIFEST["entries"] if item["artifact_kind"] == "optimize_body_fragment"]
    assert len(entries) == 8
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        fragment = parse_optimize_body_fragment(path.read_text(encoding="utf-8"), str(path))
        assert fragment.constraints or fragment.controls, entry["id"]
        assert not fragment.diagnostics, entry["id"]
        assert fragment.source_text == path.read_text(encoding="utf-8")
        assert all(record.location.line > 0 for record in fragment.recovered_records), entry["id"]
    ####


def test_every_synthetic_wrapper_is_recoverable_without_an_exception() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["wrapper_path"]]
    assert len(entries) == 149
    for entry in entries:
        path = CORPUS / entry["wrapper_path"]
        text = path.read_text()
        if entry["language"] in {"tbl", "tbl_lexical"}:
            parse_table_text(text, str(path))
        else:
            parse_problem_text(text, str(path))
    ####


def test_manual_corpus_runs_through_unified_ingestion_route() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["wrapper_path"]]
    for entry in entries:
        path = CORPUS / entry["wrapper_path"]
        text = path.read_text()
        kind = FileKind.TABLE if entry["language"] in {"tbl", "tbl_lexical"} else FileKind.PROBLEM
        result = ingest_text(text, kind=kind, source_path=str(path))

        assert result.source.render_bytes() == path.read_bytes(), entry["id"]
        assert all(diagnostic.location is not None for diagnostic in result.diagnostics), entry["id"]
    ####


def test_complete_section_45_source_displays_are_ingested_by_declared_kind() -> None:
    source_displays = sorted((Path(__file__).parents[2] / "examples" / "chapter04").glob("*-source.*.txt"))
    assert [path.name for path in source_displays] == [
        "air-launched-intercept-source.prb.txt",
        "ballistic-reentry-source.prb.txt",
        "ballistic-reentry-source.tbl.txt",
        "ballistic-rocket-source.prb.txt",
        "ground-launched-intercept-source.prb.txt",
    ]

    for path in source_displays:
        kind = FileKind.TABLE if path.name.endswith(".tbl.txt") else FileKind.PROBLEM
        result = ingest_text(path.read_text(encoding="utf-8"), kind=kind, source_path=str(path))

        assert result.valid, path.name
        assert result.source.render_bytes() == path.read_bytes(), path.name
        assert not [diagnostic for diagnostic in result.diagnostics if diagnostic.severity.value == "error"], path.name
    ####


def test_repaired_manual_constructs_have_no_syntax_diagnostics() -> None:
    expected = {
        "ch3-029",
        "ch3-030",
        "ch3-031",
        "ch4-086",
        "ch4-095",
        "ch4-111",
    }
    entries = {entry["id"]: entry for entry in MANIFEST["entries"]}
    assert expected <= entries.keys()
    for entry_id in expected:
        entry = entries[entry_id]
        path = CORPUS / entry["wrapper_path"]
        parsed = parse_table_text(path.read_text(), str(path)) if entry["language"] == "tbl" else parse_problem_text(path.read_text(), str(path))
        assert not [diagnostic for diagnostic in parsed.diagnostics if diagnostic.severity.value == "error"], entry_id
    ####


def test_manual_corpus_diagnostic_allowlist_is_explicit() -> None:
    observed: dict[str, int] = {}
    for entry in MANIFEST["entries"]:
        if not entry["wrapper_path"]:
            continue
        path = CORPUS / entry["wrapper_path"]
        parsed = parse_table_text(path.read_text(), str(path)) if entry["language"] == "tbl" else parse_problem_text(path.read_text(), str(path))
        errors = [diagnostic for diagnostic in parsed.diagnostics if diagnostic.severity.value == "error"]
        if errors:
            observed[entry["id"]] = len(errors)
            assert {diagnostic.code for diagnostic in errors} == EXPECTED_WRAPPER_DIAGNOSTIC_CODES[entry["id"]]
    assert observed == EXPECTED_WRAPPER_ERRORS
    ####


def test_manual_corpus_diagnostics_and_recovery_records_are_source_located() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["wrapper_path"]]
    for entry in entries:
        path = CORPUS / entry["wrapper_path"]
        parsed = parse_table_text(path.read_text(), str(path)) if entry["language"] in {"tbl", "tbl_lexical"} else parse_problem_text(path.read_text(), str(path))
        assert all(diagnostic.location is not None for diagnostic in parsed.diagnostics), entry["id"]
        assert all(record.location is not None for record in parsed.recovered_records), entry["id"]
        recovered_keys = {(record.code, record.location.line, record.location.column) for record in parsed.recovered_records if record.location is not None}
        assert all((diagnostic.code, diagnostic.location.line, diagnostic.location.column) in recovered_keys for diagnostic in parsed.diagnostics if diagnostic.location is not None), entry["id"]
    ####


def test_manual_corpus_recovery_records_match_their_physical_source_lines() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["wrapper_path"]]
    for entry in entries:
        path = CORPUS / entry["wrapper_path"]
        source_lines = path.read_text(encoding="utf-8").splitlines()
        parsed = parse_table_text(path.read_text(encoding="utf-8"), str(path)) if entry["language"] in {"tbl", "tbl_lexical"} else parse_problem_text(path.read_text(encoding="utf-8"), str(path))
        for record in parsed.recovered_records:
            assert record.location.line is not None
            assert record.text == source_lines[record.location.line - 1], entry["id"]
    ####


def test_manual_corpus_constructs_are_catalogued_and_baseline_is_stable() -> None:
    observed_table_types: set[str] = set()
    observed_table_operations: set[str] = set()
    observed_problem_blocks: set[str] = set()
    observed_trajectory_blocks: set[str] = set()
    observed_segment_blocks: set[str] = set()

    for entry in MANIFEST["entries"]:
        wrapper_path = entry["wrapper_path"]
        if not wrapper_path:
            continue
        path = CORPUS / wrapper_path
        if entry["language"] == "tbl":
            document = parse_table_text(path.read_text(), str(path))
            for table in document.tables:
                observed_table_types.add(table.table_type)
                observed_table_operations.update(operation.operator for operation in table.operations)
            continue
        ####
        document = parse_problem_text(path.read_text(), str(path))
        for problem in document.problems:
            observed_problem_blocks.update(block.keyword for block in problem.blocks)
            for trajectory in problem.trajectories:
                observed_trajectory_blocks.update(block.keyword for block in trajectory.blocks)
                for segment in trajectory.segments:
                    observed_segment_blocks.update(block.keyword for block in segment.blocks)
            ####
        ####
    ####

    assert observed_table_types == {"ca", "cx", "mdot", "output", "thrust"}
    assert observed_table_operations == {
        "add", "csto", "cos", "div", "end", "exp", "goto", "if", "mult", "sin", "sqr", "sub"
    }
    assert observed_problem_blocks == set(SUPPORTED_PROBLEM_BLOCKS)
    assert observed_trajectory_blocks == set(SUPPORTED_TRAJECTORY_BLOCKS)
    assert observed_segment_blocks == set(SUPPORTED_SEGMENT_BLOCKS)
    assert observed_table_types <= TABLE_TYPES
    assert observed_table_operations <= TABLE_OPERATIONS
    ####


def test_manual_negative_numeric_fixture_reports_the_invalid_comma() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-005")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))

    diagnostics = [item for item in document.diagnostics if item.code == "invalid-assignment-line"]
    assert len(diagnostics) == 1
    assert diagnostics[0].location.line == 4
    assert diagnostics[0].location.column > 1
    ####


def test_manual_negative_fly_fixture_reports_inconsistent_table_4_3_angles() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-016")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))

    diagnostics = [item for item in document.diagnostics if item.code == "inconsistent-fly-angle-set"]
    assert len(diagnostics) == 1
    assert diagnostics[0].location.line == 7
    assert any(record.code == "inconsistent-fly-angle-set" for record in document.recovered_records)
    ####


def test_manual_negative_table_fixture_reports_independent_order_error() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch3-011")
    path = CORPUS / entry["wrapper_path"]
    document = parse_table_text(path.read_text(), str(path))

    diagnostics = [item for item in document.diagnostics if item.code == "independent-assignment-order"]
    assert len(diagnostics) == 1
    assert diagnostics[0].location.line == 3
    assert any(record.code == "independent-assignment-order" for record in document.recovered_records)
    ####


def test_manual_braced_define_examples_are_typed_as_branches() -> None:
    entries = {entry["id"]: entry for entry in MANIFEST["entries"]}
    for entry_id in {"ch4-056", "ch4-057", "ch4-058"}:
        entry = entries[entry_id]
        path = CORPUS / entry["wrapper_path"]
        document = parse_problem_text(path.read_text(), str(path))
        define = next(block for block in document.problems[0].trajectories[0].blocks if block.keyword == "define")
        control = define.control_statements[0]
        assert control.body, entry_id
        assert control.body[0].location.line > define.location.line, entry_id
        if entry_id == "ch4-058":
            assert define.integral is True
            assert define.variable == "qmin"
            assert define.initial_value is not None
        if entry_id in {"ch4-057", "ch4-058"}:
            assert control.else_body, entry_id
            assert control.else_body[0].location.line > control.body[0].location.line, entry_id
    ####


def test_manual_temporary_variable_example_has_valid_assignment_order() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-053")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))

    assert not [item for item in document.diagnostics if item.code == "define-variable-used-before-assignment"]
    ####


def test_manual_fly_guidance_tables_are_typed_as_points() -> None:
    entries = {entry["id"]: entry for entry in MANIFEST["entries"]}
    for entry_id in {"ch4-024", "ch4-084", "ch4-092", "ch4-097"}:
        entry = entries[entry_id]
        path = CORPUS / entry["wrapper_path"]
        document = parse_problem_text(path.read_text(), str(path))
        fly = next(block for block in document.problems[0].trajectories[0].segments[0].blocks if block.keyword == "fly")
        assert fly.reference == "tseg", entry_id
        assert fly.points, entry_id
        assert all(point.location.line > fly.location.line for point in fly.points), entry_id
    ####


def test_manual_radar_continuation_keeps_earth_shape_and_parameters() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-101")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))
    radar = document.problems[0].blocks[0]

    assert radar.earth_shape == "wgs-72"
    assert {assignment.name for assignment in radar.assignments} >= {"distn", "diste", "distd"}
    assert not document.diagnostics
    ####


def test_manual_limits_continuation_is_typed_as_relationships() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-035")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))
    limits = document.problems[0].trajectories[0].segments[0].blocks[0]

    assert [(item.variable, item.operator) for item in limits.limits] == [
        ("bankgd", ">"), ("bankgd", "<"), ("alpha", ">"), ("alpha", "<"),
        ("beta", ">"), ("beta", "<"),
    ]
    assert not document.diagnostics
    ####


def test_manual_search_infers_trajectory_qualifier_across_endpoints() -> None:
    entry = next(item for item in MANIFEST["entries"] if item["id"] == "ch4-104")
    path = CORPUS / entry["wrapper_path"]
    document = parse_problem_text(path.read_text(), str(path))
    search = document.problems[0].blocks[0]

    assert search.objective is not None
    assert search.objective.left.trajectory == 2
    assert search.objective.right.trajectory == 2
    assert not document.diagnostics
    ####


def test_manual_lexical_sentinel_preserves_comments_and_numeric_tokens() -> None:
    path = CORPUS / "snippets/chapter03/ch3-002__full-line-and-trailing-comments.txt"
    document = lex_text(path.read_text(), source_path=str(path))
    assert document.lines[0].comment == " the entire line is used for comments"
    assert document.lines[0].tokens == ()
    assert [token.value for token in document.lines[1].tokens] == [0.2, 0.3, 0.4, 0.5]
    assert document.lines[1].comment == " Mach numbers"
    ####
