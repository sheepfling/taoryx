"""Canonical TAOS table- and problem-file language support."""

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.ebnf import EbnfGrammar, EbnfReferenceError, EbnfSyntaxError, load_ebnf, parse_ebnf
from taoryx.language.grammar_contracts import (
    SUPPORTED_GRAMMAR_CONTRACTS,
    TAOS96_FRAMING_CONTRACT,
    TAOS96_FREE_FIELD_CONTRACT,
    TAOS96_HIERARCHY_CONTRACT,
    TAOS96_PROBLEM_CATALOG_CONTRACT,
)
from taoryx.language.ingest import FileKind, IngestedDocument, ingest_file, ingest_text, kind_for_path
from taoryx.language.lexical import LexicalDocument, LexicalLine, LexicalToken, TitleTail, TokenKind, lex_line, lex_text, lex_title_tail
from taoryx.language.lossless import LosslessDocument, LosslessRecord, SourceSpan, parse_lossless_file
from taoryx.language.models import TableCall
from taoryx.language.problem_parser import parse_problem_file, parse_problem_text
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import parse_table_file, parse_table_text

__all__ = [
    "Diagnostic",
    "EbnfGrammar",
    "EbnfReferenceError",
    "EbnfSyntaxError",
    "Severity",
    "SourceLocation",
    "parse_problem_file",
    "parse_problem_text",
    "parse_table_file",
    "parse_table_text",
    "validate_problem",
    "validate_table_file",
    "LosslessDocument",
    "LosslessRecord",
    "SourceSpan",
    "parse_lossless_file",
    "load_ebnf",
    "parse_ebnf",
    "TableCall",
    "FileKind",
    "IngestedDocument",
    "ingest_file",
    "ingest_text",
    "kind_for_path",
    "LexicalDocument",
    "LexicalLine",
    "LexicalToken",
    "TitleTail",
    "TokenKind",
    "lex_line",
    "lex_text",
    "lex_title_tail",
    "SUPPORTED_GRAMMAR_CONTRACTS",
    "TAOS96_FREE_FIELD_CONTRACT",
    "TAOS96_FRAMING_CONTRACT",
    "TAOS96_HIERARCHY_CONTRACT",
    "TAOS96_PROBLEM_CATALOG_CONTRACT",
]
