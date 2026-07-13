"""Validate the tracked Chapter 3/4 manual snippet corpus as parser evidence."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any

from taoryx.language.lexical import lex_text
from taoryx.language.lossless import parse_lossless_bytes
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

ROOT = Path(__file__).resolve().parents[1]
CORPUS = ROOT / "tests" / "fixtures" / "taos_manual_corpus_v22"
# This fragment intentionally omits the tabulated data required to validate the
# ``factor`` call.  Keep the exception explicit so new parser diagnostics do
# not silently become accepted corpus behavior.
EXPECTED_WRAPPER_ERRORS = {"ch3-022": 1}
####


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()
####


def _errors(diagnostics: list[Any]) -> list[Any]:
    return [item for item in diagnostics if getattr(getattr(item, "severity", None), "value", getattr(item, "severity", None)) == "error"]
####


def _parse(language: str, text: str, path: str) -> Any:
    if language in {"tbl", "tbl_lexical"}:
        return parse_table_text(text, path)
    return parse_problem_text(text, path)
####


def main() -> int:
    manifest = json.loads((CORPUS / "manifest.json").read_text(encoding="utf-8"))
    entries = manifest["entries"]
    if len(entries) != 164 or len({entry["id"] for entry in entries}) != 164:
        raise SystemExit("Corpus manifest must contain 164 unique entries.")
    ####
    parser_exceptions = 0
    top_level_errors: list[str] = []
    wrapper_errors = Counter()
    lexical_checked = False
    for entry in entries:
        raw_path = CORPUS / entry["raw_path"]
        if not raw_path.is_file() or _sha256(raw_path) != entry["raw_sha256"]:
            raise SystemExit(f"Raw snippet hash or file mismatch: {entry['id']}")
        ####
        raw_bytes = raw_path.read_bytes()
        if parse_lossless_bytes(raw_bytes, source_path=str(raw_path)).render_bytes() != raw_bytes:
            raise SystemExit(f"Lossless round-trip failed: {entry['id']}")
        ####
        wrapper_path = entry.get("wrapper_path")
        if wrapper_path:
            wrapper = CORPUS / wrapper_path
            if not wrapper.is_file():
                raise SystemExit(f"Missing wrapper: {entry['id']}")
            try:
                parsed = _parse(entry["language"], wrapper.read_text(encoding="utf-8"), str(wrapper))
                wrapper_errors[entry["id"]] = len(_errors(parsed.diagnostics))
            except Exception as exc:  # pragma: no cover - the check is specifically an exception guard.
                parser_exceptions += 1
                raise SystemExit(f"Parser exception for {entry['id']}: {exc}") from exc
        ####
        if entry["parse_mode"] in {"top_level", "top_level_excerpt"} and entry["language"] in {"tbl", "prb"}:
            parsed = _parse(entry["language"], raw_path.read_text(encoding="utf-8"), str(raw_path))
            if _errors(parsed.diagnostics):
                top_level_errors.append(entry["id"])
        ####
        if entry["id"] == "ch3-002":
            lexical = lex_text(raw_path.read_text(encoding="utf-8"), source_path=str(raw_path))
            if len(lexical.lines) != 2 or lexical.lines[0].tokens or lexical.lines[0].comment is None or len(lexical.lines[1].tokens) != 4:
                raise SystemExit("The corpus comment/numeric lexical sentinel no longer matches.")
            lexical_checked = True
        ####
    ####
    if not lexical_checked:
        raise SystemExit("Missing ch3-002 lexical sentinel.")
    if top_level_errors:
        raise SystemExit(f"Top-level corpus parser errors: {', '.join(top_level_errors)}")
    observed_wrapper_errors = {entry_id: count for entry_id, count in wrapper_errors.items() if count}
    if observed_wrapper_errors != EXPECTED_WRAPPER_ERRORS:
        raise SystemExit(
            "Unexpected manual corpus wrapper diagnostics: "
            f"expected {dict(EXPECTED_WRAPPER_ERRORS)!r}, observed {observed_wrapper_errors!r}."
        )
    print(
        f"TAOS snippet corpus validation passed: {len(entries)} raw displays, "
        f"{len(wrapper_errors)} wrappers, {parser_exceptions} parser exceptions, "
        f"{sum(wrapper_errors.values())} wrapper diagnostics retained as evidence."
    )
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
