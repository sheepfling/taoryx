from __future__ import annotations

import json
from pathlib import Path

from taoryx.language.lexical import lex_text
from taoryx.language.lossless import parse_lossless_bytes
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

CORPUS = Path(__file__).parents[1] / "fixtures" / "taos_manual_corpus_v22"
MANIFEST = json.loads((CORPUS / "manifest.json").read_text(encoding="utf-8"))


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


def test_manual_lexical_sentinel_preserves_comments_and_numeric_tokens() -> None:
    path = CORPUS / "snippets/chapter03/ch3-002__full-line-and-trailing-comments.txt"
    document = lex_text(path.read_text(), source_path=str(path))
    assert document.lines[0].comment == " the entire line is used for comments"
    assert document.lines[0].tokens == ()
    assert [token.value for token in document.lines[1].tokens] == [0.2, 0.3, 0.4, 0.5]
    assert document.lines[1].comment == " Mach numbers"
    ####
