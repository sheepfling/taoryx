"""Validate the tracked Chapter 3/4 manual snippet corpus as parser evidence."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any

from taoryx.language._legacy_table import TABLE_OPERATIONS, TABLE_TYPES
from taoryx.language.grammar_contracts import (
    SUPPORTED_PROBLEM_BLOCKS,
    SUPPORTED_SEGMENT_BLOCKS,
    SUPPORTED_TRAJECTORY_BLOCKS,
)
from taoryx.language.lexical import lex_text
from taoryx.language.lossless import parse_lossless_bytes
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.table_parser import parse_table_text

ROOT = Path(__file__).resolve().parents[1]
CORPUS = ROOT / "tests" / "fixtures" / "taos_manual_corpus_v22"
# These are intentional negative/manual-boundary fixtures. ``ch3-022`` omits
# interpolation data required by a ``factor`` call; ``ch3-011`` preserves an
# incorrect simple-table independent-variable order; ``ch4-005`` preserves the
# manual's explicitly incorrect comma-delimited numeric value; and ``ch4-016``
# contains an explicitly inconsistent body-attitude angle set; ``ch4-051`` is
# a deliberately fragmentary trajectory wrapper without ``*initial``; and
# ``ch4-080`` is the documented ``*egs summary`` form without its prerequisite
# survey/summary blocks; ``ch4-017`` and ``ch4-026`` show the documented
# wildcard guidance examples in a trajectory's first segment. Keep all
# exceptions explicit so new parser diagnostics do not silently become
# accepted corpus behavior.
EXPECTED_WRAPPER_ERRORS = {"ch3-011": 1, "ch3-022": 1, "ch4-005": 1, "ch4-016": 1, "ch4-017": 3, "ch4-026": 3, "ch4-051": 1, "ch4-080": 2}
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
    summary = json.loads((CORPUS / "SUMMARY.json").read_text(encoding="utf-8"))
    entries = manifest["entries"]
    if len(entries) != 164 or len({entry["id"] for entry in entries}) != 164:
        raise SystemExit("Corpus manifest must contain 164 unique entries.")
    ####
    wrapper_count = sum(bool(entry.get("wrapper_path")) for entry in entries)
    if summary.get("total_displays") != len(entries) or summary.get("phase1_parser_attempted") != wrapper_count:
        raise SystemExit("Corpus SUMMARY.json display or wrapper counts do not match the manifest.")
    if summary.get("phase1_parser_exceptions") != 0:
        raise SystemExit("Corpus SUMMARY.json records parser exceptions.")
    reviewed = summary.get("source_monospace_blocks_reviewed")
    automatic = summary.get("source_monospace_automatic_matches")
    manually_classified = summary.get("source_monospace_manually_classified")
    if summary.get("source_monospace_uncaptured") != 0 or reviewed != automatic + manually_classified:
        raise SystemExit("Corpus SUMMARY.json reports uncaptured or inconsistently accounted source blocks.")
    source_pass = json.loads((CORPUS / "source_context" / "SOURCE_FINAL_PASS.json").read_text(encoding="utf-8"))
    source_audit = json.loads((CORPUS / "source_context" / "source_monospace_audit.json").read_text(encoding="utf-8"))
    if len(source_audit) != source_pass.get("source_monospace_blocks_reviewed"):
        raise SystemExit("Source monospace audit length does not match SOURCE_FINAL_PASS.json.")
    if Counter(item["review_status"] for item in source_audit) != Counter(source_pass.get("status_counts", {})):
        raise SystemExit("Source monospace audit status counts do not match SOURCE_FINAL_PASS.json.")
    manifest_ids = {entry["id"] for entry in entries}
    dangling_captures = {
        item["captured_by"]
        for item in source_audit
        if item["review_status"] in {"automatic_match", "manual_match"} and item.get("captured_by") not in manifest_ids
    }
    if dangling_captures:
        raise SystemExit(f"Source monospace audit has dangling captures: {sorted(dangling_captures)}")
    ####
    source_pdf = ROOT / "TAOS_manual_1995.pdf"
    if source_pdf.is_file() and _sha256(source_pdf) != summary.get("source_pdf_sha256"):
        raise SystemExit("Corpus source PDF hash does not match SUMMARY.json.")
    ####
    parser_exceptions = 0
    top_level_errors: list[str] = []
    wrapper_errors = Counter()
    observed_table_types: set[str] = set()
    observed_table_operations: set[str] = set()
    observed_problem_blocks: set[str] = set()
    observed_trajectory_blocks: set[str] = set()
    observed_segment_blocks: set[str] = set()
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
                wrapper_text = wrapper.read_text(encoding="utf-8")
                parsed = _parse(entry["language"], wrapper_text, str(wrapper))
                source_lines = wrapper_text.splitlines()
                for record in parsed.recovered_records:
                    location = getattr(record, "location", None)
                    if location is None or location.line < 1 or location.line > len(source_lines):
                        raise SystemExit(f"Recovery record has invalid source location for {entry['id']}")
                    if record.text != source_lines[location.line - 1]:
                        raise SystemExit(
                            f"Recovery record does not preserve its physical source line for {entry['id']} "
                            f"at line {location.line}"
                        )
                wrapper_errors[entry["id"]] = len(_errors(parsed.diagnostics))
                if entry["language"] == "tbl":
                    for table in parsed.tables:
                        observed_table_types.add(table.table_type)
                        observed_table_operations.update(operation.operator for operation in table.operations)
                else:
                    for problem in parsed.problems:
                        observed_problem_blocks.update(block.keyword for block in problem.blocks)
                        for trajectory in problem.trajectories:
                            observed_trajectory_blocks.update(block.keyword for block in trajectory.blocks)
                            for segment in trajectory.segments:
                                observed_segment_blocks.update(block.keyword for block in segment.blocks)
                            ####
                        ####
                    ####
                ####
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
    if not observed_table_types <= TABLE_TYPES:
        raise SystemExit(f"Manual corpus contains unknown table types: {sorted(observed_table_types - TABLE_TYPES)}")
    if not observed_table_operations <= TABLE_OPERATIONS:
        raise SystemExit(f"Manual corpus contains unknown table operations: {sorted(observed_table_operations - TABLE_OPERATIONS)}")
    if not observed_problem_blocks <= SUPPORTED_PROBLEM_BLOCKS:
        raise SystemExit(f"Manual corpus contains unknown problem blocks: {sorted(observed_problem_blocks - SUPPORTED_PROBLEM_BLOCKS)}")
    if not observed_trajectory_blocks <= SUPPORTED_TRAJECTORY_BLOCKS:
        raise SystemExit(f"Manual corpus contains unknown trajectory blocks: {sorted(observed_trajectory_blocks - SUPPORTED_TRAJECTORY_BLOCKS)}")
    if not observed_segment_blocks <= SUPPORTED_SEGMENT_BLOCKS:
        raise SystemExit(f"Manual corpus contains unknown segment blocks: {sorted(observed_segment_blocks - SUPPORTED_SEGMENT_BLOCKS)}")
    print(
        f"TAOS snippet corpus validation passed: {len(entries)} raw displays, "
        f"{len(wrapper_errors)} wrappers, {parser_exceptions} parser exceptions, "
        f"{sum(wrapper_errors.values())} wrapper diagnostics retained as evidence; "
        f"observed {len(observed_table_types)} table types, {len(observed_table_operations)} operations, "
        f"and {len(observed_problem_blocks | observed_trajectory_blocks | observed_segment_blocks)} problem-block forms."
    )
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
