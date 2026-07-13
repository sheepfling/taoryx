from __future__ import annotations

import re
from collections.abc import Sequence
from typing import Literal

from pydantic import BaseModel, Field

_TOKEN_RE = re.compile(
    r"\s*(?:"
    r"(?P<number>(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?)|"
    r"(?P<parameter>(?:surv|srch)-\d+|opt[a-e]-\d+)|"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_.-]*)|"
    r"(?P<operator><=|>=|==|!=|[+\-*/^<>=(),\[\]])|"
    r"(?P<wildcard>\*)"
    r")",
    re.IGNORECASE,
)


class Expression(BaseModel):
    kind: str
####


class NumberExpression(Expression):
    kind: Literal["number"] = "number"
    value: float
####


class NameExpression(Expression):
    kind: Literal["name"] = "name"
    name: str
####


class IndexedExpression(Expression):
    kind: Literal["indexed"] = "indexed"
    name: str
    index: int
####


class ParameterExpression(Expression):
    kind: Literal["parameter"] = "parameter"
    family: Literal["survey", "search", "optimize"]
    loop: str | None = None
    index: int
####


class TableReferenceExpression(Expression):
    kind: Literal["table_reference"] = "table_reference"
    name: str
####


class WildcardExpression(Expression):
    kind: Literal["wildcard"] = "wildcard"
####


class UnaryExpression(Expression):
    kind: Literal["unary"] = "unary"
    operator: str
    operand: "ExpressionType"
####


class BinaryExpression(Expression):
    kind: Literal["binary"] = "binary"
    operator: str
    left: "ExpressionType"
    right: "ExpressionType"
####


class CallExpression(Expression):
    kind: Literal["call"] = "call"
    function: str
    arguments: list["ExpressionType"] = Field(default_factory=list)
####


ExpressionType = (
    NumberExpression
    | NameExpression
    | IndexedExpression
    | ParameterExpression
    | TableReferenceExpression
    | WildcardExpression
    | UnaryExpression
    | BinaryExpression
    | CallExpression
)


class ExpressionToken(BaseModel):
    value: str
    category: str
    column: int
####


class ExpressionSyntaxError(ValueError):
    pass
####


def tokenize_expression(text: str) -> list[ExpressionToken]:
    tokens: list[ExpressionToken] = []
    position = 0
    while position < len(text):
        match = _TOKEN_RE.match(text, position)
        if match is None:
            if text[position:].strip() == "":
                break
            ####
            raise ExpressionSyntaxError(f"Unexpected expression text at column {position + 1}: {text[position:position + 20]!r}")
        ####
        category = match.lastgroup
        assert category is not None
        value = match.group(category)
        tokens.append(ExpressionToken(value=value, category=category, column=match.start(category) + 1))
        position = match.end()
    ####
    return tokens
####


class _ExpressionParser:
    _PRECEDENCE: dict[str, int] = {
        "=": 10,
        "==": 10,
        "!=": 10,
        "<": 10,
        ">": 10,
        "<=": 10,
        ">=": 10,
        "+": 20,
        "-": 20,
        "*": 30,
        "/": 30,
        "^": 40,
    }

    def __init__(self, tokens: Sequence[ExpressionToken]) -> None:
        self.tokens = tokens
        self.position = 0
    ####

    def current(self) -> ExpressionToken | None:
        if self.position >= len(self.tokens):
            return None
        ####
        return self.tokens[self.position]
    ####

    def accept(self, value: str) -> bool:
        token = self.current()
        if token is not None and token.value.lower() == value.lower():
            self.position += 1
            return True
        ####
        return False
    ####

    def parse(self) -> ExpressionType:
        expression = self.parse_binary(0)
        if self.current() is not None:
            raise ExpressionSyntaxError(f"Unexpected token {self.current().value!r} at column {self.current().column}.")
        ####
        return expression
    ####

    def parse_binary(self, minimum_precedence: int) -> ExpressionType:
        left = self.parse_prefix()
        while True:
            token = self.current()
            if token is None or token.value not in self._PRECEDENCE:
                break
            ####
            precedence = self._PRECEDENCE[token.value]
            if precedence < minimum_precedence:
                break
            ####
            operator = token.value
            self.position += 1
            next_minimum = precedence if operator == "^" else precedence + 1
            right = self.parse_binary(next_minimum)
            left = BinaryExpression(operator=operator, left=left, right=right)
        ####
        return left
    ####

    def parse_prefix(self) -> ExpressionType:
        token = self.current()
        if token is None:
            raise ExpressionSyntaxError("Unexpected end of expression.")
        ####
        if token.value in {"+", "-"}:
            self.position += 1
            return UnaryExpression(operator=token.value, operand=self.parse_prefix())
        ####
        if token.category == "number":
            self.position += 1
            return NumberExpression(value=float(token.value))
        ####
        if token.category == "parameter":
            self.position += 1
            lower = token.value.lower()
            if lower.startswith("surv-"):
                return ParameterExpression(family="survey", index=int(lower.split("-")[1]))
            ####
            if lower.startswith("srch-"):
                return ParameterExpression(family="search", index=int(lower.split("-")[1]))
            ####
            match = re.fullmatch(r"opt([a-e])-(\d+)", lower)
            assert match is not None
            return ParameterExpression(family="optimize", loop=match.group(1), index=int(match.group(2)))
        ####
        if token.category == "wildcard" or token.value == "*":
            self.position += 1
            return WildcardExpression()
        ####
        if self.accept("("):
            inner_start = self.position
            inner = self.parse_binary(0)
            if not self.accept(")"):
                raise ExpressionSyntaxError("Missing closing parenthesis.")
            ####
            if isinstance(inner, NameExpression) and self.position - inner_start == 2:
                return TableReferenceExpression(name=inner.name)
            ####
            return inner
        ####
        if token.category == "name":
            self.position += 1
            name = token.value
            if self.accept("["):
                index_token = self.current()
                if index_token is None or index_token.category != "number" or not float(index_token.value).is_integer():
                    raise ExpressionSyntaxError(f"Expected integer index after {name!r}.")
                ####
                self.position += 1
                if not self.accept("]"):
                    raise ExpressionSyntaxError("Missing closing bracket.")
                ####
                return IndexedExpression(name=name, index=int(float(index_token.value)))
            ####
            if self.accept("("):
                arguments: list[ExpressionType] = []
                if not self.accept(")"):
                    while True:
                        arguments.append(self.parse_binary(0))
                        if self.accept(")"):
                            break
                        ####
                        if not self.accept(","):
                            raise ExpressionSyntaxError("Expected comma or closing parenthesis in function call.")
                        ####
                    ####
                ####
                return CallExpression(function=name, arguments=arguments)
            ####
            return NameExpression(name=name)
        ####
        raise ExpressionSyntaxError(f"Unexpected token {token.value!r} at column {token.column}.")
    ####
####


def parse_expression(text: str) -> ExpressionType:
    return _ExpressionParser(tokenize_expression(text)).parse()
####
