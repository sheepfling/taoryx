"""Small executable reader for the repository's documentary EBNF.

This module deliberately parses the grammar notation, not TAOS source files.
It gives the documented grammar a checked, typed representation that can later
drive generated recognizers without making an unverified semantic claim today.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EbnfLocation:
    path: str
    line: int
    column: int
####


class EbnfSyntaxError(ValueError):
    """A grammar notation error with a source location."""

    def __init__(self, message: str, location: EbnfLocation) -> None:
        super().__init__(f"{location.path}:{location.line}:{location.column}: {message}")
        self.message = message
        self.location = location
    ####
####


class EbnfReferenceError(ValueError):
    """A grammar rule refers to a production that is not defined."""

    def __init__(self, name: str, location: EbnfLocation) -> None:
        super().__init__(f"{location.path}:{location.line}:{location.column}: undefined production {name!r}")
        self.name = name
        self.location = location
    ####
####


@dataclass(frozen=True)
class EbnfLiteral:
    value: str
    location: EbnfLocation
####


@dataclass(frozen=True)
class EbnfReference:
    name: str
    location: EbnfLocation
####


@dataclass(frozen=True)
class EbnfSequence:
    items: tuple["EbnfExpression", ...]
####


@dataclass(frozen=True)
class EbnfChoice:
    alternatives: tuple["EbnfExpression", ...]
####


@dataclass(frozen=True)
class EbnfOptional:
    expression: "EbnfExpression"
####


@dataclass(frozen=True)
class EbnfRepeat:
    expression: "EbnfExpression"
####


EbnfExpression = EbnfLiteral | EbnfReference | EbnfSequence | EbnfChoice | EbnfOptional | EbnfRepeat


@dataclass(frozen=True)
class EbnfRule:
    name: str
    expression: EbnfExpression
    location: EbnfLocation
####


@dataclass(frozen=True)
class EbnfGrammar:
    path: str
    rules: tuple[EbnfRule, ...]

    @property
    def rule_names(self) -> frozenset[str]:
        return frozenset(rule.name for rule in self.rules)
    ####

    def undefined_references(self) -> tuple[EbnfReference, ...]:
        references: list[EbnfReference] = []

        def walk(expression: EbnfExpression) -> None:
            if isinstance(expression, EbnfReference):
                if expression.name not in self.rule_names:
                    references.append(expression)
            elif isinstance(expression, (EbnfOptional, EbnfRepeat)):
                walk(expression.expression)
            elif isinstance(expression, (EbnfSequence, EbnfChoice)):
                for item in expression.items if isinstance(expression, EbnfSequence) else expression.alternatives:
                    walk(item)
            ####
        ####

        for rule in self.rules:
            walk(rule.expression)
        return tuple(references)
    ####

    def validate(self) -> "EbnfGrammar":
        undefined = self.undefined_references()
        if undefined:
            raise EbnfReferenceError(undefined[0].name, undefined[0].location)
        ####
        return self
    ####
####


@dataclass(frozen=True)
class _Token:
    kind: str
    value: str
    location: EbnfLocation
####


_IDENTIFIER = re.compile(r"[a-z][a-z0-9-]*")


def _tokenize(text: str, path: str) -> tuple[_Token, ...]:
    tokens: list[_Token] = []
    index = 0
    line = 1
    column = 1
    while index < len(text):
        if text.startswith("(*", index):
            end = text.find("*)", index + 2)
            if end < 0:
                raise EbnfSyntaxError("unterminated comment", EbnfLocation(path, line, column))
            comment = text[index : end + 2]
            line += comment.count("\n")
            column = len(comment.rsplit("\n", 1)[-1]) + 1 if "\n" in comment else column + len(comment)
            index = end + 2
            continue
        ####
        character = text[index]
        if character.isspace():
            if character == "\n":
                line += 1
                column = 1
            else:
                column += 1
            index += 1
            continue
        ####
        location = EbnfLocation(path, line, column)
        if character == '"':
            end = index + 1
            value: list[str] = []
            while end < len(text) and text[end] != '"':
                value.append(text[end])
                end += 1
            if end >= len(text):
                raise EbnfSyntaxError("unterminated literal", location)
            raw = text[index : end + 1]
            tokens.append(_Token("literal", "".join(value), location))
            column += len(raw)
            index = end + 1
            continue
        ####
        match = _IDENTIFIER.match(text, index)
        if match:
            value = match.group(0)
            tokens.append(_Token("identifier", value, location))
            index = match.end()
            column += len(value)
            continue
        ####
        if character in "=;|,()[]{}":
            tokens.append(_Token(character, character, location))
            index += 1
            column += 1
            continue
        ####
        raise EbnfSyntaxError(f"unexpected character {character!r}", location)
    ####
    return tuple(tokens)
####


class _Parser:
    def __init__(self, tokens: tuple[_Token, ...], path: str) -> None:
        self.tokens = tokens
        self.path = path
        self.index = 0
    ####

    def current(self) -> _Token | None:
        return self.tokens[self.index] if self.index < len(self.tokens) else None
    ####

    def take(self, kind: str) -> _Token:
        token = self.current()
        if token is None:
            raise EbnfSyntaxError(f"expected {kind!r}, reached end of grammar", EbnfLocation(self.path, 1, 1))
        if token.kind != kind:
            raise EbnfSyntaxError(f"expected {kind!r}, found {token.value!r}", token.location)
        self.index += 1
        return token
    ####

    def parse(self) -> EbnfGrammar:
        rules: list[EbnfRule] = []
        while self.current() is not None:
            name = self.take("identifier")
            self.take("=")
            rules.append(EbnfRule(name.value, self.expression({";"}), name.location))
            self.take(";")
        ####
        return EbnfGrammar(self.path, tuple(rules))
    ####

    def expression(self, stops: set[str]) -> EbnfExpression:
        alternatives = [self.sequence(stops | {"|"})]
        while self.current() is not None and self.current().kind == "|":
            self.index += 1
            alternatives.append(self.sequence(stops | {"|"}))
        ####
        return alternatives[0] if len(alternatives) == 1 else EbnfChoice(tuple(alternatives))
    ####

    def sequence(self, stops: set[str]) -> EbnfExpression:
        items: list[EbnfExpression] = []
        while self.current() is not None and self.current().kind not in stops:
            items.append(self.factor())
            if self.current() is not None and self.current().kind == ",":
                self.index += 1
            ####
        if not items:
            token = self.current()
            location = token.location if token is not None else EbnfLocation(self.path, 1, 1)
            raise EbnfSyntaxError("expected an expression", location)
        ####
        return items[0] if len(items) == 1 else EbnfSequence(tuple(items))
    ####

    def factor(self) -> EbnfExpression:
        token = self.current()
        if token is None:
            raise EbnfSyntaxError("expected an expression, reached end of grammar", EbnfLocation(self.path, 1, 1))
        if token.kind == "literal":
            self.index += 1
            return EbnfLiteral(token.value, token.location)
        if token.kind == "identifier":
            self.index += 1
            return EbnfReference(token.value, token.location)
        if token.kind in {"(", "[", "{"}:
            closing = {"(": ")", "[": "]", "{": "}"}[token.kind]
            self.index += 1
            expression = self.expression({closing})
            self.take(closing)
            if token.kind == "[":
                return EbnfOptional(expression)
            if token.kind == "{":
                return EbnfRepeat(expression)
            return expression
        raise EbnfSyntaxError(f"expected an expression, found {token.value!r}", token.location)
    ####
####


def parse_ebnf(text: str, path: str = "<memory>") -> EbnfGrammar:
    """Parse and validate EBNF text into an executable grammar model."""
    return _Parser(_tokenize(text, path), path).parse().validate()
####


def load_ebnf(path: str | Path) -> EbnfGrammar:
    source = Path(path)
    return parse_ebnf(source.read_text(encoding="utf-8"), str(source))
####
