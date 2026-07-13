from __future__ import annotations

import json
from pathlib import Path

from taoryx.language.lexical import lex_text
from taoryx.language.lossless import parse_lossless_bytes
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

CORPUS = Path(__file__).parents[1] / "fixtures" / "taos_manual_corpus_v22"
MANIFEST = json.loads((CORPUS / "manifest.json").read_text(encoding="utf-8"))
EXPECTED_WRAPPER_ERRORS = {"ch3-022": 1}


def test_corpus_manifest_and_every_raw_display_are_lossless() -> None:
    entries = MANIFEST["entries"]
    assert len(entries) == 164
    assert len({entry["id"] for entry in entries}) == 164
    for entry in entries:
        raw = (CORPUS / entry["raw_path"]).read_bytes()
        assert parse_lossless_bytes(raw, source_path=entry["raw_path"]).render_bytes() == raw
        if entry["wrapper_path"]:
            assert (CORPUS / entry["wrapper_path"]).is_file()
    ####


def test_complete_top_level_corpus_documents_have_no_parser_errors() -> None:
    entries = [entry for entry in MANIFEST["entries"] if entry["parse_mode"] in {"top_level", "top_level_excerpt"} and entry["language"] in {"tbl", "prb"}]
    assert len(entries) == 17
    for entry in entries:
        path = CORPUS / entry["raw_path"]
        parsed = parse_table_text(path.read_text(), str(path)) if entry["language"] == "tbl" else parse_problem_text(path.read_text(), str(path))
        assert not [diagnostic for diagnostic in parsed.diagnostics if diagnostic.severity.value == "error"], entry["id"]
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
    assert observed == EXPECTED_WRAPPER_ERRORS
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


def test_manual_lexical_sentinel_preserves_comments_and_numeric_tokens() -> None:
    path = CORPUS / "snippets/chapter03/ch3-002__full-line-and-trailing-comments.txt"
    document = lex_text(path.read_text(), source_path=str(path))
    assert document.lines[0].comment == " the entire line is used for comments"
    assert document.lines[0].tokens == ()
    assert [token.value for token in document.lines[1].tokens] == [0.2, 0.3, 0.4, 0.5]
    assert document.lines[1].comment == " Mach numbers"
    ####
