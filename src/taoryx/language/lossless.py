"""Lossless source preservation for TAOS text files.

This layer intentionally does not assign TAOS keyword semantics. The verified
semantic parsers remain responsible for tables and problems; these records make
source spans and byte-preserving round trips available to them.
"""

from __future__ import annotations

from pathlib import Path
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

LineEnding = Literal["", "\n", "\r\n", "\r"]
RecordKind = Literal["blank", "comment", "content"]


class SourceSpan(BaseModel):
    """Inclusive source location for one physical input line."""

    model_config = ConfigDict(frozen=True)

    path: str
    line_start: int
    line_end: int
    raw_text: str


class LosslessRecord(BaseModel):
    """One physical line without assigning language-level meaning."""

    model_config = ConfigDict(frozen=True)

    kind: RecordKind
    source: SourceSpan
    line_ending: LineEnding = ""
    inline_comment: str | None = None


class LosslessDocument(BaseModel):
    """A source document that can be rendered back to its original bytes."""

    model_config = ConfigDict(frozen=True)

    source_path: str
    encoding: str = "utf-8"
    newline: Literal["lf", "crlf", "cr", "mixed", "none"] = "none"
    final_newline: bool = False
    records: tuple[LosslessRecord, ...] = Field(default_factory=tuple)

    def render_text(self) -> str:
        return "".join(record.source.raw_text + record.line_ending for record in self.records)

    def render_bytes(self) -> bytes:
        return self.render_text().encode(self.encoding, errors="surrogateescape")


def _newline_kind(data: bytes) -> Literal["lf", "crlf", "cr", "mixed", "none"]:
    crlf = data.count(b"\r\n")
    bare_lf = data.count(b"\n") - crlf
    bare_cr = data.count(b"\r") - crlf
    kinds = sum(count > 0 for count in (crlf, bare_lf, bare_cr))
    if kinds > 1:
        return "mixed"
    if crlf:
        return "crlf"
    if bare_lf:
        return "lf"
    if bare_cr:
        return "cr"
    return "none"


def _split_physical_lines(data: bytes) -> list[tuple[bytes, bytes]]:
    lines: list[tuple[bytes, bytes]] = []
    for line in data.splitlines(keepends=True):
        if line.endswith(b"\r\n"):
            lines.append((line[:-2], b"\r\n"))
        elif line.endswith((b"\n", b"\r")):
            lines.append((line[:-1], line[-1:]))
        else:
            lines.append((line, b""))
    return lines


def _record_kind(raw_text: str) -> RecordKind:
    stripped = raw_text.strip()
    if not stripped:
        return "blank"
    if stripped.startswith("#"):
        return "comment"
    return "content"


def parse_lossless_bytes(
    data: bytes,
    *,
    source_path: str = "<memory>",
    encoding: str = "utf-8",
) -> LosslessDocument:
    physical_lines = _split_physical_lines(data)
    records = tuple(
        LosslessRecord(
            kind=_record_kind(raw_text := raw_line.decode(encoding, errors="surrogateescape")),
            source=SourceSpan(
                path=source_path,
                line_start=line_number,
                line_end=line_number,
                raw_text=raw_text,
            ),
            line_ending=line_ending.decode("ascii"),
        )
        for line_number, (raw_line, line_ending) in enumerate(physical_lines, start=1)
    )
    return LosslessDocument(
        source_path=source_path,
        encoding=encoding,
        newline=_newline_kind(data),
        final_newline=data.endswith((b"\n", b"\r")),
        records=records,
    )


def parse_lossless_text(text: str, *, source_path: str = "<memory>") -> LosslessDocument:
    return parse_lossless_bytes(text.encode("utf-8"), source_path=source_path)


def parse_lossless_file(path: str | Path, *, encoding: str = "utf-8") -> LosslessDocument:
    source = Path(path)
    return parse_lossless_bytes(source.read_bytes(), source_path=str(source), encoding=encoding)
