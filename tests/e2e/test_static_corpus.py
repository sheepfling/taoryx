from __future__ import annotations

import json

from taoryx.language.ingest import ingest_file

from .live_compatibility import LIVE_POSITIVE_DIAGNOSTICS
from .support.loader import load_manifest, load_metamorphic, repository_root
from .support.static_validation import validate_case

EXPECTED_DIAGNOSTIC_ALIASES = {
    "nonmonotonic-independent-values": frozenset({"unordered-independent-values"}),
    "inconsistent-guidance-angle-set": frozenset({"inconsistent-fly-angle-set"}),
    "atmosphere-altitude-not-strict": frozenset({"duplicate-atmos-altitude"}),
    "invalid-parameter-index": frozenset({"nonsequential-optimize-parameters"}),
}
EXPECTED_POSITIVE_INGEST_WARNINGS = {
    "p017_radar_relative": frozenset({"ambiguous-dual-scope-block"}),
    "p048_problem_level_outputs_and_egs": frozenset({"ambiguous-dual-scope-block"}),
}


def test_v23_manifest_and_case_files_are_complete() -> None:
    root = repository_root()
    manifest_payload = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    cases = load_manifest()

    assert len(cases) == 63
    assert len({case.id for case in cases}) == len(cases)
    assert not [case.id for case in cases if case.static_expectation == "future-error"]
    assert len(manifest_payload["cases"]) == len(cases)
    manifest_paths = {case.path for case in cases}
    case_directories = {
        path.relative_to(root).as_posix()
        for path in (root / "cases").glob("*")
        if path.is_dir()
        for path in path.glob("*")
        if path.is_dir()
    }
    assert case_directories == manifest_paths
    referenced_inputs = {
        (case.path + "/" + case.problem_file)
        for case in cases
    } | {
        case.path + "/" + relative
        for case in cases
        for relative in case.table_files
    }
    corpus_inputs = {
        path.relative_to(root).as_posix()
        for path in (root / "cases").rglob("*")
        if path.is_file() and path.suffix in {".prb", ".tbl"}
    }
    assert corpus_inputs == referenced_inputs
    for case in cases:
        directory = root / case.path
        assert (directory / "case.yaml").is_file()
        assert (directory / case.problem_file).is_file()
        assert all((directory / relative).is_file() for relative in case.table_files)
    ####


def test_v23_metamorphic_groups_reference_known_cases() -> None:
    case_ids = {case.id for case in load_manifest()}
    groups = load_metamorphic()

    assert len(groups) == 9
    assert all(set(group.cases) <= case_ids for group in groups)
    ####


def test_v23_files_are_lossless_and_every_diagnostic_has_recovery() -> None:
    for case in load_manifest():
        directory = repository_root() / case.path
        table_types: dict[str, str] = {}
        for relative in case.table_files:
            table_path = directory / relative
            table_result = ingest_file(table_path)
            assert table_result.source.render_bytes() == table_path.read_bytes()
            for table in table_result.document.tables:
                table_types[table.name.casefold()] = table.table_type
            ####
            for diagnostic in table_result.diagnostics:
                assert diagnostic.location is not None, table_path
                assert any(
                    record.code == diagnostic.code
                    and record.location == diagnostic.location
                    for record in table_result.document.recovered_records
                ), (table_path, diagnostic.code, diagnostic.location)
            ####
        ####
        problem_path = directory / case.problem_file
        problem_result = ingest_file(problem_path, available_tables=table_types)
        assert problem_result.source.render_bytes() == problem_path.read_bytes()
        for diagnostic in problem_result.diagnostics:
            assert diagnostic.location is not None, problem_path
            assert any(
                record.code == diagnostic.code
                and record.location == diagnostic.location
                for record in problem_result.document.recovered_records
            ), (problem_path, diagnostic.code, diagnostic.location)
        ####
        if case.kind == "positive":
            observed_warnings = frozenset(
                diagnostic.code
                for diagnostic in problem_result.diagnostics
                if diagnostic.severity.value == "warning"
            )
            assert observed_warnings <= EXPECTED_POSITIVE_INGEST_WARNINGS.get(case.id, frozenset()), case.id
        ####


def test_positive_cases_are_static_clean_or_explicitly_classified() -> None:
    for case in load_manifest():
        if case.kind != "positive":
            continue
        ####
        result = validate_case(case)
        assert all(diagnostic.location is not None for diagnostic in result.diagnostics), case.id
        errors = frozenset(result.error_codes)
        allowed = LIVE_POSITIVE_DIAGNOSTICS.get(case.id, frozenset())
        assert errors <= allowed, (case.id, sorted(errors), sorted(allowed))
        if case.id not in LIVE_POSITIVE_DIAGNOSTICS:
            assert not errors, (case.id, result.diagnostic_codes)
        ####
    ####


def test_negative_cases_produce_located_diagnostics() -> None:
    for case in load_manifest():
        if case.kind != "negative":
            continue
        ####
        result = validate_case(case)
        assert all(diagnostic.location is not None for diagnostic in result.diagnostics), case.id
        observed = set(result.error_codes)
        for expected in case.expected_diagnostics:
            accepted = {expected} | set(EXPECTED_DIAGNOSTIC_ALIASES.get(expected, ()))
            assert observed & accepted, (case.id, expected, sorted(observed))
        ####
        if case.static_expectation == "future-error":
            # These are intentionally retained semantic-gap witnesses. They
            # may already produce a different diagnostic, or none yet.
            continue
        ####
        assert result.error_codes, case.id
        ####
    ####
