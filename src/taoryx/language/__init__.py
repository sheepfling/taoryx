"""Canonical TAOS table- and problem-file language support."""

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import parse_table_file, parse_table_text

__all__ = [
    "Diagnostic",
    "Severity",
    "SourceLocation",
    "parse_problem_file",
    "parse_problem_text",
    "parse_table_file",
    "parse_table_text",
    "validate_problem",
    "validate_table_file",
]
