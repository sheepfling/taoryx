"""Unified ingestion and validation entrypoint for TAOS text files."""

from __future__ import annotations

from enum import StrEnum
from pathlib import Path

from pydantic import BaseModel, ConfigDict

from taoryx.language.diagnostics import Diagnostic, Severity
from taoryx.language.lexical import LexicalDocument, lex_text
from taoryx.language.lossless import LosslessDocument, parse_lossless_bytes
from taoryx.language.models import ProblemDocument, TableDocument
from taoryx.language.problem_parser import parse_problem_text
from taoryx.language.semantic_validation import validate_problem, validate_table_file
from taoryx.language.table_parser import parse_table_text


class FileKind(StrEnum):
    """Supported TAOS source-file families."""

    PROBLEM = "prb"
    TABLE = "tbl"


class UnsupportedFileKindError(ValueError):
    """Raised when a path does not identify a supported TAOS file family."""


class IngestedDocument(BaseModel):
    """Lossless source plus current verified semantic interpretation."""

    model_config = ConfigDict(arbitrary_types_allowed=True, frozen=True)

    kind: FileKind
    source: LosslessDocument
    lexical: LexicalDocument
    document: ProblemDocument | TableDocument
    diagnostics: tuple[Diagnostic, ...] = ()

    @property
    def valid(self) -> bool:
        return not any(diagnostic.severity is Severity.ERROR for diagnostic in self.diagnostics)


def kind_for_path(path: str | Path) -> FileKind:
    suffix = Path(path).suffix.casefold()
    if suffix == ".prb":
        return FileKind.PROBLEM
    if suffix == ".tbl":
        return FileKind.TABLE
    raise UnsupportedFileKindError(f"unsupported TAOS file extension: {suffix or '<none>'}")


def ingest_text(
    text: str,
    *,
    kind: FileKind,
    source_path: str = "<memory>",
    available_tables: set[str] | None = None,
) -> IngestedDocument:
    source = parse_lossless_bytes(text.encode("utf-8"), source_path=source_path)
    lexical = lex_text(text, source_path=source_path)
    if kind is FileKind.PROBLEM:
        document = parse_problem_text(text, source_path)
        diagnostics = validate_problem(document, available_tables=available_tables)
    else:
        document = parse_table_text(text, source_path)
        diagnostics = validate_table_file(document)
    return IngestedDocument(
        kind=kind,
        source=source,
        lexical=lexical,
        document=document,
        diagnostics=tuple(diagnostics),
    )


def ingest_file(
    path: str | Path,
    *,
    available_tables: set[str] | None = None,
    encoding: str = "utf-8",
) -> IngestedDocument:
    source_path = Path(path)
    data = source_path.read_bytes()
    text = data.decode(encoding, errors="surrogateescape")
    source = parse_lossless_bytes(data, source_path=str(source_path), encoding=encoding)
    lexical = lex_text(text, source_path=str(source_path))
    kind = kind_for_path(source_path)
    if kind is FileKind.PROBLEM:
        document = parse_problem_text(text, str(source_path))
        diagnostics = validate_problem(document, available_tables=available_tables)
    else:
        document = parse_table_text(text, str(source_path))
        diagnostics = validate_table_file(document)
    return IngestedDocument(kind=kind, source=source, lexical=lexical, document=document, diagnostics=tuple(diagnostics))
