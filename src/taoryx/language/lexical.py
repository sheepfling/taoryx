"""Lossless-aware lexical analysis for the supported TAOS free-field syntax."""

from __future__ import annotations

import re
from enum import StrEnum

from pydantic import BaseModel, ConfigDict, Field

from taoryx.language.lossless import SourceSpan


class TokenKind(StrEnum):
    IDENTIFIER = "identifier"
    INTEGER = "integer"
    REAL = "real"
    SCIENTIFIC = "scientific"
    SURVEY_REFERENCE = "survey-reference"
    SEARCH_REFERENCE = "search-reference"
    OPTIMIZATION_REFERENCE = "optimization-reference"
    STRING = "string"
    PUNCTUATION = "punctuation"


class LexicalToken(BaseModel):
    model_config = ConfigDict(frozen=True)

    kind: TokenKind
    raw: str
    normalized: str
    value: str | int | float
    line: int
    column: int
    offset_start: int
    offset_end: int


class LexicalLine(BaseModel):
    model_config = ConfigDict(frozen=True)

    source: SourceSpan
    content: str
    comment: str | None = None
    tokens: tuple[LexicalToken, ...] = ()


class LexicalDocument(BaseModel):
    model_config = ConfigDict(frozen=True)

    source_path: str
    lines: tuple[LexicalLine, ...] = Field(default_factory=tuple)


class TitleTail(BaseModel):
    """A case-preserving remainder of a title-bearing record."""

    model_config = ConfigDict(frozen=True)

    source: SourceSpan
    text: str


_INTEGER_RE = re.compile(r"^[+-]?\d+$")
_REAL_RE = re.compile(r"^[+-]?(?:(?:\d+\.\d*)|(?:\.\d+))$")
_SCIENTIFIC_RE = re.compile(r"^[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))[EeDd][+-]?\d+$")
_SURVEY_RE = re.compile(r"^surv-([1-9]\d*)$", re.IGNORECASE)
_SEARCH_RE = re.compile(r"^srch-([1-9]\d*)$", re.IGNORECASE)
_OPTIMIZATION_RE = re.compile(r"^opt([a-z])-([1-9]\d*)$", re.IGNORECASE)
_PUNCTUATION = frozenset(",=():<>;")


def _classify(raw: str, line: int, column: int, start: int) -> LexicalToken:
    normalized = raw.lower()
    common = dict(raw=raw, normalized=normalized, line=line, column=column, offset_start=start, offset_end=start + len(raw))
    if _SURVEY_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.SURVEY_REFERENCE, value=normalized, **common)
    if _SEARCH_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.SEARCH_REFERENCE, value=normalized, **common)
    if _OPTIMIZATION_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.OPTIMIZATION_REFERENCE, value=normalized, **common)
    if _INTEGER_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.INTEGER, value=int(raw), **common)
    if _SCIENTIFIC_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.SCIENTIFIC, value=float(raw.replace("D", "E").replace("d", "e")), **common)
    if _REAL_RE.fullmatch(raw):
        return LexicalToken(kind=TokenKind.REAL, value=float(raw), **common)
    if raw in _PUNCTUATION:
        return LexicalToken(kind=TokenKind.PUNCTUATION, value=raw, **common)
    return LexicalToken(kind=TokenKind.IDENTIFIER, value=normalized, **common)
####


def lex_line(raw_text: str, *, line: int = 1, source_path: str = "<memory>") -> LexicalLine:
    quote: str | None = None
    comment_start = -1
    for position, character in enumerate(raw_text):
        if character in {"'", '"'}:
            quote = None if quote == character else character if quote is None else quote
        elif character == "#" and quote is None:
            comment_start = position
            break
    content = raw_text if comment_start < 0 else raw_text[:comment_start]
    comment = None if comment_start < 0 else raw_text[comment_start + 1 :]
    tokens: list[LexicalToken] = []
    index = 0
    while index < len(content):
        if content[index].isspace():
            index += 1
            continue
        if content[index] in {"'", '"'}:
            quote = content[index]
            end = index + 1
            while end < len(content) and content[end] != quote:
                end += 1
            if end < len(content):
                end += 1
            raw = content[index:end]
            tokens.append(
                LexicalToken(
                    kind=TokenKind.STRING,
                    raw=raw,
                    normalized=raw,
                    value=raw[1:-1] if raw.endswith(quote) else raw[1:],
                    line=line,
                    column=index + 1,
                    offset_start=index,
                    offset_end=end,
                )
            )
            index = end
            continue
        if content[index] in _PUNCTUATION:
            tokens.append(_classify(content[index], line, index + 1, index))
            index += 1
            continue
        end = index + 1
        while end < len(content) and content[end] not in _PUNCTUATION and not content[end].isspace() and content[end] not in {"'", '"'}:
            end += 1
        tokens.append(_classify(content[index:end], line, index + 1, index))
        index = end
    return LexicalLine(
        source=SourceSpan(path=source_path, line_start=line, line_end=line, raw_text=raw_text),
        content=content,
        comment=comment,
        tokens=tuple(tokens),
    )
####


def lex_text(text: str, *, source_path: str = "<memory>") -> LexicalDocument:
    return LexicalDocument(
        source_path=source_path,
        lines=tuple(lex_line(line, line=line_number, source_path=source_path) for line_number, line in enumerate(text.splitlines(), start=1)),
    )
####


def lex_title_tail(text: str, *, line: int = 1, source_path: str = "<memory>") -> TitleTail:
    start = len(text) - len(text.lstrip())
    return TitleTail(
        source=SourceSpan(path=source_path, line_start=line, line_end=line, raw_text=text[start:]),
        text=text[start:],
    )
####
