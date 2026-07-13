from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.problem_parser import parse_problem_file
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import parse_table_file

from .loader import case_directory
from .models import CaseSpec


@dataclass(slots=True)
class StaticResult:
    diagnostics: list[Diagnostic]
    diagnostic_codes: list[str]
    error_codes: list[str]
    warning_codes: list[str]
####


def validate_case(case: CaseSpec, root: Path | None = None) -> StaticResult:
    directory = case_directory(case, root)
    table_names: set[str] = set()
    diagnostics: list[Diagnostic] = []
    diagnostic_codes: list[str] = []
    error_codes: list[str] = []
    warning_codes: list[str] = []
    for relative in case.table_files:
        document = parse_table_file(directory / relative)
        for table in document.tables:
            normalized_name = table.name.casefold()
            if normalized_name in table_names:
                diagnostic_codes.append("duplicate-table")
                error_codes.append("duplicate-table")
            else:
                table_names.add(normalized_name)
            ####
        ####
        for diagnostic in validate_table_file(document):
            diagnostics.append(diagnostic)
            diagnostic_codes.append(diagnostic.code)
            if diagnostic.severity == "error":
                error_codes.append(diagnostic.code)
            elif diagnostic.severity == "warning":
                warning_codes.append(diagnostic.code)
            ####
        ####
    ####
    problem_path = directory / case.problem_file
    problem_document = parse_problem_file(problem_path)
    for diagnostic in validate_problem(problem_document, available_tables=table_names):
        diagnostics.append(diagnostic)
        diagnostic_codes.append(diagnostic.code)
        if diagnostic.severity == "error":
            error_codes.append(diagnostic.code)
        elif diagnostic.severity == "warning":
            warning_codes.append(diagnostic.code)
        ####
    ####
    # Close the current parser's empty-table-set dependency gap.
    source = problem_path.read_bytes().decode("utf-8", errors="surrogateescape")
    for match in re.finditer(r"=\(([^()]+)\)", source):
        reference = match.group(1).casefold()
        if reference in table_names:
            continue
        ####
        line = source.count("\n", 0, match.start()) + 1
        line_start = source.rfind("\n", 0, match.start()) + 1
        diagnostics.append(
            Diagnostic(
                severity=Severity.ERROR,
                code="external-table-reference",
                message=f"Referenced table {match.group(1)!r} is not present in the case table files; source text was preserved.",
                location=SourceLocation(path=str(problem_path), line=line, column=match.start() - line_start + 1),
            )
        )
        diagnostic_codes.append("external-table-reference")
        error_codes.append("external-table-reference")
        ####
    ####
    return StaticResult(diagnostics, diagnostic_codes, error_codes, warning_codes)
####
