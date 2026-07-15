from __future__ import annotations

import re
from collections import Counter
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field

from taoryx.language.grammar_contracts import DOCUMENTED_STATE_VARIABLES

NUMBER_RE = re.compile(r"^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?$")
# TAOS table names and labels use a wider historical name alphabet than
# Python identifiers (for example, the manual's ``1st-stage`` table).  Values
# used as variables in calls and independent-variable lists remain
# letter/underscore-led.
IDENTIFIER_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]*$")
VARIABLE_IDENTIFIER_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]*$")
OPTION_ATOM_RE = re.compile(r"^[A-Za-z_][A-Za-z0-9_./-]*$")
TABLE_NAME_RE = re.compile(r"^\(([^()]+)\)$")
BLOCK_RE = re.compile(r"^\*(?P<keyword>[A-Za-z0-9_/]+)\b(?P<header>.*)$")
ASSIGNMENT_RE = re.compile(
    r"(?P<name>[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)\s*"
    r"(?P<operator>=|<|>)\s*"
    r"(?P<value>\([^()]+\)|\*|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?|[A-Za-z_][A-Za-z0-9_./-]*(?:\[\d+\])?)"
)
TABLE_REFERENCE_RE = re.compile(r"=\s*\(([^()]+)\)")
OPT_REFERENCE_RE = re.compile(r"\bopt([a-e])-(\d+)\b", re.IGNORECASE)
SURVEY_REFERENCE_RE = re.compile(r"\bsurv-(\d+)\b", re.IGNORECASE)
SEARCH_REFERENCE_RE = re.compile(r"\bsrch-(\d+)\b", re.IGNORECASE)
INDEXED_VARIABLE_RE = re.compile(r"\b[A-Za-z_][A-Za-z0-9_./-]*\[(\d+)\]")
RELATIONSHIP_RE = re.compile(
    r"^\s*(?:[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?|[A-Za-z_][A-Za-z0-9_./-]*)"
    r"\s*(?:<|>|=)\s*"
    r"(?:[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][+-]?\d+)?|[A-Za-z_][A-Za-z0-9_./-]*)\s*$"
)


def _is_operation_label(value: str) -> bool:
    """Return whether a label uses the manual's name-or-value vocabulary."""
    return IDENTIFIER_RE.fullmatch(value) is not None or NUMBER_RE.fullmatch(value) is not None
####

TABLE_OPERATIONS_WITH_OPERAND = {
    "add",
    "sub",
    "mult",
    "div",
    "idiv",
    "exp",
    "iexp",
    "min",
    "max",
    "set",
    "csto",
    "goto",
}
TABLE_OPERATIONS_WITHOUT_OPERAND = {
    "abs",
    "neg",
    "sqr",
    "sqrt",
    "ln",
    "log",
    "e",
    "sin",
    "cos",
    "tan",
    "asin",
    "acos",
    "atan",
    "zero",
    "end",
}
TABLE_OPERATIONS = TABLE_OPERATIONS_WITH_OPERAND | TABLE_OPERATIONS_WITHOUT_OPERAND | {"if"}
TABLE_TYPES = {
    "ca",
    "cn",
    "cl",
    "cd",
    "cs",
    "cx",
    "cy",
    "cz",
    "thrust",
    "tvec1",
    "tvec2",
    "mdot",
    "cg",
    "windv",
    "windh",
    "winde",
    "windn",
    "windd",
    "output",
}
TAORYX_TABLE_TYPES = {"cmx", "cmy", "cmz"}
COEFFICIENT_TABLE_TYPES = {"ca", "cn", "cl", "cd", "cs", "cx", "cy", "cz", *TAORYX_TABLE_TYPES}
THRUST_UNITS = {"lb", "n", "kn"}
MASS_FLOW_UNITS = {
    "lb/sec",
    "lb/min",
    "lb/hr",
    "slugs/sec",
    "slugs/min",
    "slugs/hr",
    "g/sec",
    "g/min",
    "g/hr",
    "kg/sec",
    "kg/min",
    "kg/hr",
}
LIMITED_STATE_VARIABLES = {
    "alt", "altdt", "dynprs", "gamgc", "gamgd", "latgc", "latgd",
    "long", "mass", "nu", "pres", "psigc", "rcm", "rcmdt", "rho", "sndspd", "temp",
    "time", "tmark", "vel", "wt",
}
LIMITED_STATE_TABLE_TYPES = {"cg", "windv", "windh", "winde", "windn", "windd"}
PROBLEM_LEVEL_BLOCKS = {
    "title",
    "atmos",
    "define",
    "earth",
    "egs",
    "file",
    "optimize",
    "print",
    "radar",
    "search",
    "summarize",
    "survey",
    "units/fmt",
    "wind",
    "end",
}
TRAJECTORY_LEVEL_BLOCKS = {
    "define",
    "dwn/crs",
    "file",
    "iip",
    "initial",
    "print",
    "tangent",
}
SEGMENT_LEVEL_BLOCKS = {
    "aero",
    "constants",
    "cg",
    "fly",
    "increment",
    "inertial",
    "integ",
    "limits",
    "prop",
    "rail",
    "reset",
    "when",
}


class ParseIssue(BaseModel):
    severity: Literal["error", "warning", "note"]
    code: str
    message: str
    line: int | None = None
    column: int | None = None
    ####
####


class Token(BaseModel):
    value: str
    line: int
    column: int
    ####
####


class NumericAssignment(BaseModel):
    name: str
    values: list[float]
    line: int
    ####
####


class TableCall(BaseModel):
    name: str
    arguments: list[str] = Field(default_factory=list)
    parenthesized: bool = False
    ####
####


class TableOperation(BaseModel):
    operator: str
    operand: str | float | TableCall | None = None
    line: int
    assignments: list[NumericAssignment] = Field(default_factory=list)
    condition: str | None = None
    label: str | None = None
    extrapolation: str | None = None
    nested: "TableOperation | None" = None
    ####
####


class TaosTable(BaseModel):
    name: str
    table_type: str
    format: Literal["simple", "full"]
    header_line: int
    independent_variables: list[str] = Field(default_factory=list)
    options: dict[str, str | float] = Field(default_factory=dict)
    assignments: list[NumericAssignment] = Field(default_factory=list)
    operations: list[TableOperation] = Field(default_factory=list)
    omissions: list[int] = Field(default_factory=list)
    source_path: str
    ####
####


class TableParseResult(BaseModel):
    path: str
    tables: list[TaosTable]
    issues: list[ParseIssue]
    executable_complete: bool
    ####
####


class ProblemBlock(BaseModel):
    keyword: str
    header: str
    line: int
    scope: Literal["problem", "trajectory", "segment"]
    body_lines: list[str] = Field(default_factory=list)
    ####
####


class SegmentNode(BaseModel):
    number: int
    title: str
    line: int
    blocks: list[ProblemBlock] = Field(default_factory=list)
    ####
####


class TrajectoryNode(BaseModel):
    number: int
    name: str
    start_segment: int
    line: int
    blocks: list[ProblemBlock] = Field(default_factory=list)
    segments: list[SegmentNode] = Field(default_factory=list)
    ####
####


class TaosProblem(BaseModel):
    name: str
    path: str
    problem_blocks: list[ProblemBlock] = Field(default_factory=list)
    trajectories: list[TrajectoryNode] = Field(default_factory=list)
    ended: bool = False
    ####
####


class ProblemParseResult(BaseModel):
    path: str
    problem: TaosProblem | None
    issues: list[ParseIssue]
    table_references: list[str] = Field(default_factory=list)
    ####
####


def _strip_comment(line: str) -> str:
    return line.split("#", 1)[0]
####


def _validate_table_options(table_type: str, options: dict[str, str | float], line: int, issues: list[ParseIssue]) -> None:
    """Apply the table-parameter rules documented in Chapter 3."""
    allowed_parameter = "sref" if table_type in COEFFICIENT_TABLE_TYPES else "units" if table_type in {"thrust", "mdot"} else None
    for name, value in options.items():
        if name == "extrapolation":
            continue
        ####
        if name != allowed_parameter:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="unsupported-table-option",
                    message=f"Table type {table_type!r} does not document the {name!r} header parameter.",
                    line=line,
                )
            )
            continue
        ####
        if name == "sref":
            if not isinstance(value, float):
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-table-sref",
                        message="The sref table parameter requires a numeric value.",
                        line=line,
                    )
                )
        elif name == "units":
            units = str(value).casefold()
            allowed_units = THRUST_UNITS if table_type == "thrust" else MASS_FLOW_UNITS
            if units not in allowed_units:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="unsupported-table-units",
                        message=f"Units {value!r} are not documented for table type {table_type!r}.",
                        line=line,
                    )
                )
        ####
    ####
####


def _tokenize_table(text: str) -> list[Token]:
    tokens: list[Token] = []
    punctuation = set("()=:<>[]")
    for line_number, original_line in enumerate(text.splitlines(), start=1):
        line = _strip_comment(original_line)
        index = 0
        while index < len(line):
            character = line[index]
            if character.isspace() or character == ",":
                index += 1
                continue
            ####
            if character in punctuation:
                tokens.append(Token(value=character, line=line_number, column=index + 1))
                index += 1
                continue
            ####
            start = index
            while index < len(line) and not line[index].isspace() and line[index] not in punctuation and line[index] != ",":
                index += 1
            ####
            tokens.append(Token(value=line[start:index], line=line_number, column=start + 1))
        ####
        tokens.append(Token(value="\n", line=line_number, column=len(line) + 1))
    ####
    return tokens
####


class _TableTokenParser:
    def __init__(self, tokens: list[Token], path: Path) -> None:
        self.tokens = tokens
        self.path = path
        self.position = 0
        self.issues: list[ParseIssue] = []
    ####

    def current(self) -> Token | None:
        if self.position >= len(self.tokens):
            return None
        ####
        return self.tokens[self.position]
    ####

    def peek(self, offset: int = 1) -> Token | None:
        position = self.position + offset
        if position >= len(self.tokens):
            return None
        ####
        return self.tokens[position]
    ####

    def advance(self) -> Token | None:
        token = self.current()
        if token is not None:
            self.position += 1
        ####
        return token
    ####

    def skip_newlines(self) -> None:
        while self.current() is not None and self.current().value == "\n":
            self.position += 1
        ####
    ####

    def accept(self, value: str) -> Token | None:
        token = self.current()
        if token is not None and token.value.lower() == value.lower():
            self.position += 1
            return token
        ####
        return None
    ####

    def expect(self, value: str) -> Token | None:
        token = self.accept(value)
        if token is None:
            current = self.current()
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="expected-token",
                    message=f"Expected {value!r}, found {current.value!r}" if current else f"Expected {value!r} at end of file.",
                    line=current.line if current else None,
                    column=current.column if current else None,
                )
            )
        ####
        return token
    ####

    def parse_atom(self) -> str | float | None:
        token = self.current()
        if token is None:
            return None
        ####
        if NUMBER_RE.match(token.value):
            self.advance()
            return float(token.value)
        ####
        value = token.value
        self.advance()
        return value
    ####

    def parse_table_call(self) -> TableCall | None:
        token = self.current()
        if token is None or token.value in {"\n", "(", ")", "=", ":", "<", ">"}:
            return None
        ####
        name = token.value
        self.advance()
        if self.accept("(") is None:
            return TableCall(name=name, arguments=[])
        ####
        arguments: list[str] = []
        while self.current() is not None and self.current().value != ")":
            if self.current().value != "\n":
                arguments.append(self.current().value)
            ####
            self.advance()
        ####
        self.expect(")")
        return TableCall(name=name, arguments=arguments, parenthesized=True)
    ####

    def _looks_like_assignment(self) -> bool:
        current = self.current()
        following = self.peek()
        return current is not None and following is not None and current.value not in {"\n"} and following.value == "="
    ####

    def _looks_like_operation(self) -> bool:
        token = self.current()
        if token is None:
            return False
        ####
        value = token.value.lower()
        if value in TABLE_OPERATIONS:
            return True
        ####
        following = self.peek()
        after = self.peek(2)
        return following is not None and following.value == ":" and after is not None and after.value.lower() in TABLE_OPERATIONS
    ####

    def parse_numeric_assignment(self) -> NumericAssignment | None:
        if not self._looks_like_assignment():
            return None
        ####
        name_token = self.advance()
        if name_token is None:
            return None
        if VARIABLE_IDENTIFIER_RE.fullmatch(name_token.value) is None:
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-table-assignment-name",
                    message=f"Table assignment name {name_token.value!r} must be an identifier.",
                    line=name_token.line,
                    column=name_token.column,
                )
            )
        ####
        self.expect("=")
        values: list[float] = []
        while self.current() is not None:
            token = self.current()
            if token.value == "\n":
                following_position = self.position + 1
                while following_position < len(self.tokens) and self.tokens[following_position].value == "\n":
                    following_position += 1
                ####
                if following_position >= len(self.tokens):
                    self.position = following_position
                    break
                ####
                possible = self.tokens[following_position]
                possible_next = self.tokens[following_position + 1] if following_position + 1 < len(self.tokens) else None
                if (
                    possible.value.lower() in TABLE_OPERATIONS
                    or possible.value == "("
                    or possible.value == "."
                    or possible.value.lower() == "etc."
                    or (possible_next is not None and possible_next.value in {"=", ":"})
                ):
                    self.position = following_position
                    break
                ####
                self.position = following_position
                continue
            ####
            if self._looks_like_assignment() or self._looks_like_operation() or token.value.lower() == "end" or token.value == "." or token.value.lower() == "etc.":
                break
            ####
            if NUMBER_RE.match(token.value):
                values.append(float(token.value))
            else:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="nonnumeric-table-value",
                        message=f"Table assignment {name_token.value!r} contains nonnumeric value {token.value!r}.",
                        line=token.line,
                        column=token.column,
                    )
                )
            ####
            self.advance()
        ####
        if not values:
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="empty-table-assignment",
                    message=f"Table assignment {name_token.value!r} has no numeric values.",
                    line=name_token.line,
                    column=name_token.column,
                )
            )
        ####
        return NumericAssignment(name=name_token.value.lower(), values=values, line=name_token.line)
    ####

    def parse_operation(self) -> TableOperation | None:
        self.skip_newlines()
        token = self.current()
        if token is None:
            return None
        ####
        if token.value.lower() == "end":
            return None
        ####
        label: str | None = None
        if self.peek() is not None and self.peek().value == ":":
            label = token.value
            if not _is_operation_label(label):
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-operation-label",
                        message=f"Operation label {label!r} must be an identifier.",
                        line=token.line,
                        column=token.column,
                    )
                )
            self.advance()
            self.advance()
            token = self.current()
        ####
        if token is None or token.value.lower() not in TABLE_OPERATIONS:
            return None
        ####
        operator = token.value.lower()
        line = token.line
        self.advance()
        condition: str | None = None
        operand: str | float | TableCall | None = None
        extrapolation: str | None = None
        if operator == "if":
            condition_tokens: list[str] = []
            depth = 0
            if self.accept("(") is not None:
                depth = 1
                while self.current() is not None and depth > 0:
                    current = self.advance()
                    if current is None:
                        break
                    if current.value == "(":
                        depth += 1
                    elif current.value == ")":
                        depth -= 1
                    ####
                    if depth > 0 and current.value != "\n":
                        condition_tokens.append(current.value)
                    ####
                ####
            else:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-if-condition",
                        message="If operation requires a parenthesized relationship.",
                        line=line,
                        column=token.column,
                    )
                )
            ####
            condition_text = " ".join(condition_tokens)
            if not RELATIONSHIP_RE.fullmatch(condition_text):
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-if-condition",
                        message="Full-table if conditions must contain one simple relation: value <|>|= value.",
                        line=line,
                        column=token.column,
                    )
                )
            ####
            if self.accept("then") is None:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="missing-if-then",
                        message="Full-table if operation requires the 'then' keyword.",
                        line=line,
                        column=token.column,
                    )
                )
            nested = self.parse_operation()
            if nested is None:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-if-operation",
                        message="If statement is missing its then-operation.",
                        line=line,
                        column=token.column,
                    )
                )
                return TableOperation(operator=operator, line=line, condition=" ".join(condition_tokens), label=label)
            ####
            if nested.operator == "if":
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-if-operation",
                        message="A full-table if statement may contain a math operation, not a nested if statement.",
                        line=line,
                        column=token.column,
                    )
                )
            ####
            return TableOperation(operator=operator, line=line, condition=condition_text, label=label, nested=nested)
        ####
        if operator in TABLE_OPERATIONS_WITH_OPERAND:
            call = self.parse_table_call()
            if call is not None:
                if call.parenthesized and (
                    not call.arguments
                    or any(VARIABLE_IDENTIFIER_RE.fullmatch(argument) is None for argument in call.arguments)
                ):
                    self.issues.append(
                        ParseIssue(
                            severity="error",
                            code="invalid-table-call",
                            message="Table calls require one or more identifier arguments.",
                            line=line,
                            column=token.column,
                        )
                    )
                ####
                if call.parenthesized and len(call.arguments) > 5:
                    self.issues.append(
                        ParseIssue(
                            severity="error",
                            code="too-many-table-call-arguments",
                            message="A tabulated table call may contain at most five independent variables.",
                            line=line,
                            column=token.column,
                        )
                    )
                ####
                if operator == "goto" and not call.arguments:
                    operand = call.name
                elif not call.arguments and NUMBER_RE.match(call.name):
                    operand = float(call.name)
                elif not call.arguments:
                    operand = call.name
                else:
                    operand = call
                ####
            ####
            if operand is None:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="missing-operation-operand",
                        message=f"Operation {operator!r} requires a value, table call, or variable operand.",
                        line=line,
                        column=token.column,
                    )
                )
            ####
            if operator in {"csto", "goto"}:
                valid_operand = (
                    isinstance(operand, str)
                    and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_.-]*", operand) is not None
                    if operator == "csto"
                    else isinstance(operand, str) and _is_operation_label(operand)
                )
                if not valid_operand:
                    self.issues.append(
                        ParseIssue(
                            severity="error",
                            code="invalid-operation-operand",
                            message=f"Operation {operator!r} requires a destination/name identifier operand.",
                            line=line,
                            column=token.column,
                        )
                    )
            ####
        ####
        if self.current() is not None and self.current().value.lower() in {"extrap", "no-extrap"}:
            extrapolation = self.current().value.lower()
            self.advance()
        ####
        assignments: list[NumericAssignment] = []
        while True:
            self.skip_newlines()
            assignment = self.parse_numeric_assignment()
            if assignment is None:
                break
            ####
            assignments.append(assignment)
        ####
        return TableOperation(
            operator=operator,
            operand=operand,
            line=line,
            assignments=assignments,
            condition=condition,
            label=label,
            extrapolation=extrapolation,
        )
    ####

    def parse_table(self) -> TaosTable | None:
        self.skip_newlines()
        if self.current() is None:
            return None
        ####
        if self.accept("(") is None:
            token = self.advance()
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-table-name",
                    message=f"Expected table identification name, found {token.value!r}." if token else "Missing table name.",
                    line=token.line if token else None,
                    column=token.column if token else None,
                )
            )
            return None
        ####
        name_token = self.advance()
        if name_token is None:
            return None
        ####
        if IDENTIFIER_RE.fullmatch(name_token.value) is None:
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-table-name",
                    message=f"Table name {name_token.value!r} must be an identifier.",
                    line=name_token.line,
                    column=name_token.column,
                )
            )
        ####
        self.expect(")")
        self.skip_newlines()
        table_token = self.expect("table")
        if table_token is None:
            return None
        ####
        type_token = self.advance()
        if type_token is None:
            return None
        ####
        table_type = type_token.value.lower()
        if table_type not in TABLE_TYPES | TAORYX_TABLE_TYPES:
            self.issues.append(
                ParseIssue(
                    severity="error",
                    code="unknown-table-type",
                    message=f"Unknown table type {table_type!r}.",
                    line=type_token.line,
                    column=type_token.column,
                )
            )
        ####
        independent_variables: list[str] = []
        if self.accept("(") is not None:
            list_token = self.current()
            saw_variable = False
            while self.current() is not None and self.current().value != ")":
                if self.current().value != "\n":
                    saw_variable = True
                    variable = self.current()
                    independent_variables.append(variable.value.lower())
                    if VARIABLE_IDENTIFIER_RE.fullmatch(variable.value) is None:
                        self.issues.append(
                            ParseIssue(
                                severity="error",
                                code="invalid-independent-variable",
                                message=f"Independent variable {variable.value!r} must be an identifier.",
                                line=variable.line,
                                column=variable.column,
                            )
                        )
                ####
                self.advance()
            ####
            if not saw_variable:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="empty-independent-variable-list",
                        message="A parenthesized table independent-variable list cannot be empty.",
                        line=list_token.line if list_token is not None else name_token.line,
                        column=list_token.column if list_token is not None else name_token.column,
                    )
                )
            ####
            self.expect(")")
            if len(independent_variables) > 5:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="too-many-independent-variables",
                        message="A table may declare at most five independent variables.",
                        line=name_token.line,
                        column=name_token.column,
                    )
                )
        ####
        options: dict[str, str | float] = {}
        while self.current() is not None:
            if self.current().value == "\n":
                self.advance()
                break
            ####
            if self.current().value.lower() in {"extrap", "no-extrap"}:
                option_token = self.current()
                option = option_token.value.lower()
                previous = options.get("extrapolation")
                if previous is not None and previous != option:
                    self.issues.append(
                        ParseIssue(
                            severity="error",
                            code="conflicting-table-extrapolation",
                            message="A table header may specify either 'extrap' or 'no-extrap', not both.",
                            line=option_token.line,
                            column=option_token.column,
                        )
                    )
                else:
                    options["extrapolation"] = option
                self.advance()
                continue
            ####
            if self._looks_like_assignment():
                key = self.advance()
                if key is None:
                    continue
                self.expect("=")
                value = self.parse_atom()
                if isinstance(value, float) or (isinstance(value, str) and OPTION_ATOM_RE.fullmatch(value) is not None):
                    options[key.value.lower()] = value
                else:
                    self.issues.append(
                        ParseIssue(
                            severity="error",
                            code="invalid-table-option-value",
                            message=f"Table option {key.value!r} requires a numeric or identifier atom.",
                            line=key.line,
                            column=key.column,
                        )
                    )
                    if value == "(":
                        while self.current() is not None and self.current().value not in {")", "\n"}:
                            self.advance()
                        ####
                        self.accept(")")
                    ####
                ####
                continue
            ####
            self.issues.append(
                ParseIssue(
                    severity="warning",
                    code="unparsed-table-header-token",
                    message=f"Unparsed table-header token {self.current().value!r}.",
                    line=self.current().line,
                    column=self.current().column,
                )
            )
            self.advance()
        ####
        _validate_table_options(table_type, options, name_token.line, self.issues)
        self.skip_newlines()
        format_name: Literal["simple", "full"] = "simple"
        if self.current() is not None and self.current().value.lower() == "start" and not self._looks_like_assignment():
            self.advance()
            format_name = "full"
        assignments: list[NumericAssignment] = []
        operations: list[TableOperation] = []
        omissions: list[int] = []
        if format_name == "simple":
            while self.current() is not None:
                self.skip_newlines()
                if self.current() is None:
                    break
                ####
                if self.current().value == "(":
                    break
                ####
                if self.current().value.startswith(".") or self.current().value.lower() == "etc.":
                    omissions.append(self.current().line)
                    self.advance()
                    continue
                ####
                assignment = self.parse_numeric_assignment()
                if assignment is not None:
                    assignments.append(assignment)
                    continue
                ####
                token = self.advance()
                self.issues.append(
                    ParseIssue(
                        severity="warning",
                        code="unparsed-simple-table-token",
                    message=f"Unparsed simple-table token {token.value!r}." if token else "Unparsed token.",
                    line=token.line if token else None,
                    column=token.column if token else None,
                    )
                )
            ####
        else:
            saw_end = False
            while self.current() is not None:
                self.skip_newlines()
                if self.current() is None:
                    break
                ####
                if self.current().value.startswith(".") or self.current().value.lower() == "etc.":
                    omissions.append(self.current().line)
                    self.advance()
                    continue
                ####
                if self.current().value.lower() == "end":
                    end_token = self.advance()
                    if end_token is None:
                        break
                    operations.append(TableOperation(operator="end", line=end_token.line))
                    saw_end = True
                    break
                ####
                if self._looks_like_assignment() and operations:
                    assignment = self.parse_numeric_assignment()
                    if assignment is not None:
                        operations[-1].assignments.append(assignment)
                        continue
                    ####
                ####
                operation = self.parse_operation()
                if operation is not None:
                    operations.append(operation)
                    if operation.operator == "end":
                        saw_end = True
                        break
                    continue
                ####
                token = self.advance()
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="unparsed-full-table-token",
                    message=f"Unparsed full-table token {token.value!r}." if token else "Unparsed token.",
                    line=token.line if token else None,
                    column=token.column if token else None,
                    )
                )
            ####
            if not saw_end:
                self.issues.append(
                    ParseIssue(
                        severity="error",
                        code="missing-full-table-end",
                        message=f"Full table {name_token.value!r} is missing its final 'end' operation.",
                        line=name_token.line,
                        column=name_token.column,
                    )
                )
            else:
                trailing_reported = False
                while self.current() is not None and self.current().value != "(":
                    token = self.current()
                    if token.value != "\n" and not trailing_reported:
                        self.issues.append(
                            ParseIssue(
                                severity="error",
                                code="trailing-full-table-text",
                                message=f"Full table {name_token.value!r} has text after its final 'end' operation; source was preserved.",
                                line=token.line,
                                column=token.column,
                            )
                        )
                        trailing_reported = True
                    self.advance()
                ####
        ####
        return TaosTable(
            name=name_token.value,
            table_type=table_type,
            format=format_name,
            header_line=name_token.line,
            independent_variables=independent_variables,
            options=options,
            assignments=assignments,
            operations=operations,
            omissions=sorted(set(omissions)),
            source_path=str(self.path),
        )
    ####

    def parse_all(self) -> list[TaosTable]:
        tables: list[TaosTable] = []
        while True:
            self.skip_newlines()
            if self.current() is None:
                break
            ####
            start_position = self.position
            table = self.parse_table()
            if table is None:
                if self.position == start_position:
                    self.advance()
                while self.current() is not None and self.current().value != "(":
                    self.advance()
                continue
            ####
            tables.append(table)
        ####
        return tables
    ####
####


def _validate_monotonic(values: list[float]) -> Literal["increasing", "decreasing", "duplicate", "unordered", "constant"]:
    if len(values) <= 1:
        return "constant"
    ####
    differences = [right - left for left, right in zip(values, values[1:])]
    if all(value > 0 for value in differences):
        return "increasing"
    ####
    if all(value < 0 for value in differences):
        return "decreasing"
    ####
    if any(value == 0 for value in differences):
        return "duplicate"
    ####
    return "unordered"
####


def _semantic_validate_table(table: TaosTable, issues: list[ParseIssue]) -> None:
    independent_names = {name.casefold() for name in table.independent_variables}
    if table.table_type.casefold() in independent_names:
        issues.append(
            ParseIssue(
                severity="error",
                code="self-referential-table",
                message=f"Table {table.name!r} cannot use its dependent variable {table.table_type!r} as an independent variable.",
                line=table.header_line,
            )
        )
    ####
    if table.table_type.casefold() in LIMITED_STATE_TABLE_TYPES:
        for variable in table.independent_variables:
            normalized = variable.casefold()
            if normalized in DOCUMENTED_STATE_VARIABLES and normalized not in LIMITED_STATE_VARIABLES:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="unsupported-limited-state-variable",
                        message=f"Table type {table.table_type!r} may not depend on non-limited state variable {variable!r}; user-defined variables remain permitted.",
                        line=table.header_line,
                    )
                )
            ####
        ####
    if table.format == "simple":
        by_name = {assignment.name: assignment for assignment in table.assignments}
        seen_assignments: dict[str, NumericAssignment] = {}
        for assignment in table.assignments:
            previous = seen_assignments.get(assignment.name)
            if previous is not None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="duplicate-table-assignment",
                        message=(
                            f"Simple table {table.name!r} assigns {assignment.name!r} more than once; "
                            "the duplicate values are preserved without choosing one."
                        ),
                        line=assignment.line,
                    )
                )
            ####
            seen_assignments[assignment.name] = assignment
        ####
        provided_independent = [assignment.name for assignment in table.assignments if assignment.name in table.independent_variables]
        declared_independent = [variable for variable in table.independent_variables if variable in by_name]
        if len(provided_independent) == len(declared_independent) and provided_independent != declared_independent:
            first_misordered = next(
                assignment
                for index, assignment in enumerate(table.assignments)
                if assignment.name in table.independent_variables
                and (
                    index >= len(declared_independent)
                    or assignment.name != declared_independent[len([item for item in table.assignments[: index + 1] if item.name in table.independent_variables]) - 1]
                )
            )
            issues.append(
                ParseIssue(
                    severity="error",
                    code="independent-assignment-order",
                    message=f"Simple table {table.name!r} assigns independent variables in a different order than its header declaration.",
                    line=first_misordered.line,
                )
            )
        ####
        for variable in table.independent_variables:
            if variable not in by_name:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="missing-independent-values",
                        message=f"Table {table.name!r} is missing values for independent variable {variable!r}.",
                        line=table.header_line,
                    )
                )
            ####
        ####
        dependent = by_name.get(table.table_type)
        if dependent is None:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-dependent-values",
                    message=f"Simple table {table.name!r} is missing dependent values named {table.table_type!r}.",
                    line=table.header_line,
                )
            )
            return
        ####
        expected = 1
        for variable in table.independent_variables:
            assignment = by_name.get(variable)
            if assignment is not None:
                expected *= len(assignment.values)
                ordering = _validate_monotonic(assignment.values)
                if ordering == "duplicate":
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="duplicate-independent-values",
                            message=f"Independent variable {variable!r} in simple table {table.name!r} contains duplicate values.",
                            line=assignment.line,
                        )
                    )
                elif ordering == "unordered":
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="unordered-independent-values",
                            message=f"Independent variable {variable!r} in simple table {table.name!r} is not monotonic.",
                            line=assignment.line,
                        )
                    )
                ####
            ####
        ####
        if expected != len(dependent.values):
            issues.append(
                ParseIssue(
                    severity="error",
                    code="simple-table-cardinality",
                    message=f"Table {table.name!r} has {len(dependent.values)} dependent values; expected {expected}.",
                    line=dependent.line,
                )
            )
        ####
        return
    ####
    if not table.operations:
        issues.append(
            ParseIssue(
                severity="error",
                code="empty-full-table",
                message=f"Full table {table.name!r} contains no math operations.",
                line=table.header_line,
            )
        )
        return
    ####
    labels: dict[str, TableOperation] = {}
    storage_names: dict[str, TableOperation] = {}
    goto_operations: list[TableOperation] = []

    def inspect_control_flow(operation: TableOperation) -> None:
        if operation.label is not None:
            label = operation.label.casefold()
            previous = labels.get(label)
            if previous is not None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="duplicate-operation-label",
                        message=f"Full-table operation label {operation.label!r} is defined more than once; the destination is ambiguous.",
                        line=operation.line,
                    )
                )
            else:
                labels[label] = operation
        ####
        if operation.operator == "csto" and isinstance(operation.operand, str):
            storage_name = operation.operand.casefold()
            if storage_name in DOCUMENTED_STATE_VARIABLES:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="reserved-storage-variable",
                        message=f"Storage variable {operation.operand!r} is reserved for a documented TAOS state variable.",
                        line=operation.line,
                    )
                )
            ####
            previous = storage_names.get(storage_name)
            if previous is not None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="duplicate-storage-variable",
                        message=f"Storage variable {operation.operand!r} is stored more than once; its value is ambiguous.",
                        line=operation.line,
                    )
                )
            else:
                storage_names[storage_name] = operation
        ####
        if operation.operator == "goto" and isinstance(operation.operand, str):
            goto_operations.append(operation)
        ####
        if operation.nested is not None:
            inspect_control_flow(operation.nested)
        ####

    for operation in table.operations:
        inspect_control_flow(operation)
    for operation in goto_operations:
        if operation.operand is not None and operation.operand.casefold() not in labels:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="undefined-operation-label",
                    message=f"Goto destination {operation.operand!r} has no matching full-table operation label.",
                    line=operation.line,
                )
            )
    ####
    for operation in table.operations:
        if not isinstance(operation.operand, TableCall) or not operation.operand.arguments:
            continue
        ####
        dependent_name = operation.operand.name.lower()
        independent_names = [name.lower() for name in operation.operand.arguments]
        groups: list[list[NumericAssignment]] = []
        current_group: list[NumericAssignment] = []
        for assignment in operation.assignments:
            current_group.append(assignment)
            if assignment.name == dependent_name:
                groups.append(current_group)
                current_group = []
            ####
        ####
        if current_group:
            groups.append(current_group)
        ####
        if not groups:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-interpolation-data",
                    message=f"Operation {operation.operator!r} in table {table.name!r} calls {dependent_name!r} without tabulated data.",
                    line=operation.line,
                )
            )
            continue
        ####
        for group in groups:
            by_name = {assignment.name: assignment for assignment in group}
            seen_assignments: dict[str, NumericAssignment] = {}
            for assignment in group:
                previous = seen_assignments.get(assignment.name)
                if previous is not None:
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="duplicate-interpolation-assignment",
                            message=(
                                f"Interpolation data for {dependent_name!r} assigns {assignment.name!r} more than once; "
                                "the duplicate values are preserved without choosing one."
                            ),
                            line=assignment.line,
                        )
                    )
                ####
                seen_assignments[assignment.name] = assignment
            ####
            group_independent_names = [assignment.name for assignment in group if assignment.name != dependent_name]
            if set(group_independent_names) == set(independent_names) and group_independent_names != independent_names:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="interpolation-assignment-order",
                        message=f"Interpolation data for {dependent_name!r} lists independent variables in a different order than the table call.",
                        line=group[0].line,
                    )
                )
            ####
            if dependent_name not in by_name:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="missing-group-dependent-values",
                        message=f"Interpolation group in table {table.name!r} lacks dependent variable {dependent_name!r}.",
                        line=group[0].line,
                    )
                )
                continue
            ####
            expected = 1
            for independent_name in independent_names:
                assignment = by_name.get(independent_name)
                if assignment is None:
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="missing-group-independent-values",
                            message=f"Interpolation group in table {table.name!r} lacks independent variable {independent_name!r}.",
                            line=group[0].line,
                        )
                    )
                    continue
                ####
                expected *= len(assignment.values)
                ordering = _validate_monotonic(assignment.values)
                if ordering == "duplicate":
                    severity: Literal["error", "warning", "note"] = "warning"
                    code = "duplicate-independent-value"
                    message = (
                        f"Independent variable {independent_name!r} in table {table.name!r} contains a duplicate abscissa. "
                        "The source manual says duplicates are not allowed; this fixture preserves the printed example."
                    )
                    issues.append(ParseIssue(severity=severity, code=code, message=message, line=assignment.line))
                elif ordering == "unordered":
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="unordered-independent-values",
                            message=f"Independent variable {independent_name!r} in table {table.name!r} is not monotonic.",
                            line=assignment.line,
                        )
                    )
                ####
            ####
            dependent_assignment = by_name[dependent_name]
            if expected != len(dependent_assignment.values):
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="full-table-cardinality",
                        message=(
                            f"Interpolation group in table {table.name!r} has {len(dependent_assignment.values)} "
                            f"dependent values; expected {expected}."
                        ),
                        line=dependent_assignment.line,
                    )
                )
            ####
        ####
    ####
####


def parse_table_file(path: Path) -> TableParseResult:
    parser = _TableTokenParser(_tokenize_table(path.read_text(encoding="utf-8")), path)
    tables = parser.parse_all()
    issues = list(parser.issues)
    for table in tables:
        _semantic_validate_table(table, issues)
    ####
    executable_complete = bool(tables) and not any(table.omissions for table in tables)
    if not executable_complete:
        issues.append(
            ParseIssue(
                severity="note",
                code="documentation-excerpt",
                message="The fixture contains an explicit omission marker and is documentation-only rather than executable-complete.",
            )
        )
    ####
    return TableParseResult(path=str(path), tables=tables, issues=issues, executable_complete=executable_complete)
####


def _block_scope(keyword: str, current_trajectory: TrajectoryNode | None, current_segment: SegmentNode | None) -> Literal["problem", "trajectory", "segment"]:
    if keyword in SEGMENT_LEVEL_BLOCKS:
        return "segment"
    ####
    if keyword in TRAJECTORY_LEVEL_BLOCKS and current_segment is None and current_trajectory is not None:
        return "trajectory"
    ####
    if keyword in PROBLEM_LEVEL_BLOCKS:
        return "problem"
    ####
    if current_segment is not None:
        return "segment"
    ####
    if current_trajectory is not None:
        return "trajectory"
    ####
    return "problem"
####


def _parse_trajectory_header(header: str, line: int, issues: list[ParseIssue]) -> tuple[int, str, int] | None:
    match = re.match(r"\s*(\d+)\s+(.+?)\s+start\s+on\s+(\d+)\s*$", header, re.IGNORECASE)
    if match is None:
        issues.append(
            ParseIssue(
                severity="error",
                code="invalid-trajectory-header",
                message=f"Could not parse *trajectory header {header!r}.",
                line=line,
            )
        )
        return None
    ####
    return int(match.group(1)), match.group(2).strip(), int(match.group(3))
####


def _parse_segment_header(header: str, line: int, issues: list[ParseIssue]) -> tuple[int, str] | None:
    match = re.match(r"\s*(\d+)\s*(.*?)\s*$", header)
    if match is None:
        issues.append(
            ParseIssue(
                severity="error",
                code="invalid-segment-header",
                message=f"Could not parse *segment header {header!r}.",
                line=line,
            )
        )
        return None
    ####
    return int(match.group(1)), match.group(2).strip()
####


def parse_problem_file(path: Path) -> ProblemParseResult:
    lines = path.read_text(encoding="utf-8").splitlines()
    issues: list[ParseIssue] = []
    problem_name: str | None = None
    for line_number, original_line in enumerate(lines, start=1):
        stripped = _strip_comment(original_line).strip()
        if not stripped:
            continue
        ####
        match = TABLE_NAME_RE.match(stripped)
        if match is not None:
            problem_name = match.group(1)
            break
        ####
        issues.append(
            ParseIssue(
                severity="error",
                code="missing-problem-name",
                message="First noncomment statement must be a parenthesized problem identification name.",
                line=line_number,
            )
        )
        break
    ####
    if problem_name is None:
        return ProblemParseResult(path=str(path), problem=None, issues=issues)
    ####
    problem = TaosProblem(name=problem_name, path=str(path))
    current_trajectory: TrajectoryNode | None = None
    current_segment: SegmentNode | None = None
    current_block: ProblemBlock | None = None
    seen_name = False
    for line_number, original_line in enumerate(lines, start=1):
        without_comment = _strip_comment(original_line)
        stripped = without_comment.strip()
        if not stripped:
            continue
        ####
        if not seen_name and TABLE_NAME_RE.match(stripped):
            seen_name = True
            continue
        ####
        block_match = BLOCK_RE.match(stripped)
        if block_match is None:
            if current_block is None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="orphan-problem-statement",
                        message=f"Statement {stripped!r} is not attached to a data block.",
                        line=line_number,
                    )
                )
            else:
                current_block.body_lines.append(stripped)
            ####
            continue
        ####
        keyword = block_match.group("keyword").lower()
        header = block_match.group("header").strip()
        if keyword == "trajectory":
            parsed = _parse_trajectory_header(header, line_number, issues)
            if parsed is None:
                current_trajectory = None
                current_segment = None
                current_block = None
                continue
            ####
            number, name, start_segment = parsed
            current_trajectory = TrajectoryNode(
                number=number,
                name=name,
                start_segment=start_segment,
                line=line_number,
            )
            problem.trajectories.append(current_trajectory)
            current_segment = None
            current_block = None
            continue
        ####
        if keyword == "segment":
            if current_trajectory is None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="segment-without-trajectory",
                        message="A *segment block appears before any *trajectory block.",
                        line=line_number,
                    )
                )
                continue
            ####
            parsed_segment = _parse_segment_header(header, line_number, issues)
            if parsed_segment is None:
                continue
            ####
            segment_number, title = parsed_segment
            current_segment = SegmentNode(number=segment_number, title=title, line=line_number)
            current_trajectory.segments.append(current_segment)
            current_block = None
            continue
        ####
        if keyword == "end":
            problem.ended = True
            block = ProblemBlock(keyword=keyword, header=header, line=line_number, scope="problem")
            problem.problem_blocks.append(block)
            current_block = block
            current_segment = None
            current_trajectory = None
            continue
        ####
        scope = _block_scope(keyword, current_trajectory, current_segment)
        block = ProblemBlock(keyword=keyword, header=header, line=line_number, scope=scope)
        if scope == "segment":
            if current_segment is None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="segment-block-outside-segment",
                        message=f"*{keyword} is a segment block but no segment is active.",
                        line=line_number,
                    )
                )
                problem.problem_blocks.append(block)
            else:
                current_segment.blocks.append(block)
            ####
        elif scope == "trajectory":
            if current_trajectory is None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="trajectory-block-outside-trajectory",
                        message=f"*{keyword} is a trajectory block but no trajectory is active.",
                        line=line_number,
                    )
                )
                problem.problem_blocks.append(block)
            else:
                current_trajectory.blocks.append(block)
            ####
            current_segment = None
        else:
            problem.problem_blocks.append(block)
            if keyword in PROBLEM_LEVEL_BLOCKS:
                current_segment = None
            ####
        ####
        current_block = block
    ####
    _semantic_validate_problem(problem, lines, issues)
    table_references = sorted(set(TABLE_REFERENCE_RE.findall("\n".join(lines))))
    return ProblemParseResult(path=str(path), problem=problem, issues=issues, table_references=table_references)
####


def _all_problem_text(problem: TaosProblem) -> str:
    path = Path(problem.path)
    return path.read_text(encoding="utf-8")
####


def _semantic_validate_problem(problem: TaosProblem, lines: list[str], issues: list[ParseIssue]) -> None:
    if not problem.ended:
        issues.append(
            ParseIssue(
                severity="error",
                code="missing-end-block",
                message="Problem file does not contain *end.",
            )
        )
    ####
    trajectory_numbers = [trajectory.number for trajectory in problem.trajectories]
    duplicates = sorted(number for number, count in Counter(trajectory_numbers).items() if count > 1)
    for number in duplicates:
        issues.append(
            ParseIssue(
                severity="error",
                code="duplicate-trajectory-number",
                message=f"Trajectory number {number} is defined more than once.",
            )
        )
    ####
    trajectory_map = {trajectory.number: trajectory for trajectory in problem.trajectories}
    for trajectory in problem.trajectories:
        segment_numbers = [segment.number for segment in trajectory.segments]
        duplicate_segments = sorted(number for number, count in Counter(segment_numbers).items() if count > 1)
        for number in duplicate_segments:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="duplicate-segment-number",
                    message=f"Trajectory {trajectory.number} defines segment {number} more than once.",
                )
            )
        ####
        segment_map = {segment.number: segment for segment in trajectory.segments}
        if trajectory.start_segment not in segment_map:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-start-segment",
                    message=f"Trajectory {trajectory.number} starts on undefined segment {trajectory.start_segment}.",
                    line=trajectory.line,
                )
            )
        ####
        for segment in trajectory.segments:
            when_blocks = [block for block in segment.blocks if block.keyword == "when"]
            if not when_blocks:
                issues.append(
                    ParseIssue(
                        severity="warning",
                        code="segment-without-termination",
                        message=f"Trajectory {trajectory.number}, segment {segment.number} has no *when termination block.",
                        line=segment.line,
                    )
                )
            ####
            for block in when_blocks:
                goto_match = re.search(r"\bgoto\s+(\d+)\b", block.header, re.IGNORECASE)
                if goto_match is not None and int(goto_match.group(1)) not in segment_map:
                    issues.append(
                        ParseIssue(
                            severity="error",
                            code="undefined-goto-segment",
                            message=(
                                f"Trajectory {trajectory.number}, segment {segment.number} jumps to undefined "
                                f"segment {goto_match.group(1)}."
                            ),
                            line=block.line,
                        )
                    )
                ####
            ####
        ####
        initial_blocks = [block for block in trajectory.blocks if block.keyword == "initial"]
        if not initial_blocks:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-initial-block",
                    message=f"Trajectory {trajectory.number} has no *initial block.",
                    line=trajectory.line,
                )
            )
        ####
        for block in initial_blocks:
            inherited = re.search(r"from\s+trajectory\s+(\d+)\s*,?\s*segment\s+(\d+)", block.header, re.IGNORECASE)
            if inherited is None:
                continue
            ####
            source_trajectory = int(inherited.group(1))
            source_segment = int(inherited.group(2))
            target = trajectory_map.get(source_trajectory)
            if target is None:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="undefined-initial-trajectory",
                        message=f"Inherited initial state references undefined trajectory {source_trajectory}.",
                        line=block.line,
                    )
                )
            elif source_segment not in {segment.number for segment in target.segments}:
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="undefined-initial-segment",
                        message=(
                            f"Inherited initial state references undefined segment {source_segment} "
                            f"of trajectory {source_trajectory}."
                        ),
                        line=block.line,
                    )
                )
            ####
        ####
    ####
    full_text = "\n".join(lines)
    for index_match in INDEXED_VARIABLE_RE.finditer(full_text):
        trajectory_number = int(index_match.group(1))
        if trajectory_number not in trajectory_map:
            line = full_text[: index_match.start()].count("\n") + 1
            issues.append(
                ParseIssue(
                    severity="error",
                    code="undefined-indexed-trajectory",
                    message=f"Indexed variable references undefined trajectory {trajectory_number}.",
                    line=line,
                )
            )
        ####
    ####
    optimize_blocks = [block for block in problem.problem_blocks if block.keyword == "optimize"]
    optimize_by_loop: dict[str, ProblemBlock] = {}
    for block in optimize_blocks:
        match = re.match(
            r"\s*([a-e])\s+for\s+([A-Za-z_][A-Za-z0-9_./-]*)\s*=\s*(min|max)\s+on\s+segment\s+(\d+)\s*,?\s*trajectory\s+(\d+)",
            block.header,
            re.IGNORECASE,
        )
        if match is None:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-optimize-header",
                    message=f"Could not parse *optimize header {block.header!r}.",
                    line=block.line,
                )
            )
            continue
        ####
        loop = match.group(1).lower()
        optimize_by_loop[loop] = block
        segment_number = int(match.group(4))
        trajectory_number = int(match.group(5))
        trajectory = trajectory_map.get(trajectory_number)
        if trajectory is None or segment_number not in {segment.number for segment in trajectory.segments}:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-optimize-objective-location",
                    message=f"Optimization objective references undefined trajectory/segment {trajectory_number}/{segment_number}.",
                    line=block.line,
                )
            )
        ####
    ####
    opt_references: dict[str, set[int]] = {}
    for match in OPT_REFERENCE_RE.finditer(full_text):
        opt_references.setdefault(match.group(1).lower(), set()).add(int(match.group(2)))
    ####
    for loop, parameter_numbers in opt_references.items():
        block = optimize_by_loop.get(loop)
        if block is None:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-optimize-block",
                    message=f"Input references optimization loop {loop!r}, but no matching *optimize block exists.",
                )
            )
            continue
        ####
        definition_text = "\n".join(block.body_lines)
        defined = {int(value) for value in re.findall(r"\bpar-(\d+)\s*=", definition_text, re.IGNORECASE)}
        missing = sorted(parameter_numbers - defined)
        if missing:
            issues.append(
                ParseIssue(
                    severity="error",
                    code="undefined-optimization-parameter",
                    message=f"Optimization loop {loop!r} references parameters without par-N definitions: {missing}.",
                    line=block.line,
                )
            )
        ####
    ####
    survey_blocks = {
        int(match.group(1))
        for block in problem.problem_blocks
        if block.keyword == "survey"
        for match in [re.match(r"\s*(\d+)\b", block.header)]
        if match is not None
    }
    survey_references = {int(value) for value in SURVEY_REFERENCE_RE.findall(full_text)}
    for number in sorted(survey_references - survey_blocks):
        issues.append(
            ParseIssue(
                severity="error",
                code="missing-survey-block",
                message=f"Input references survey {number}, but no matching *survey block exists.",
            )
        )
    ####
    search_blocks = {
        int(match.group(1))
        for block in problem.problem_blocks
        if block.keyword == "search"
        for match in [re.match(r"\s*(\d+)\b", block.header)]
        if match is not None
    }
    search_references = {int(value) for value in SEARCH_REFERENCE_RE.findall(full_text)}
    for number in sorted(search_references - search_blocks):
        issues.append(
            ParseIssue(
                severity="error",
                code="missing-search-block",
                message=f"Input references search {number}, but no matching *search block exists.",
            )
        )
    ####
    for block in [item for trajectory in problem.trajectories for segment in trajectory.segments for item in segment.blocks]:
        if block.keyword == "when":
            if not re.search(r"(?:=|<|>)", block.header):
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-when-condition",
                        message=f"*when header lacks a comparison: {block.header!r}.",
                        line=block.line,
                    )
                )
            ####
            if not re.search(r"\b(?:goto\s+\d+|stop)\b", block.header, re.IGNORECASE):
                issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-when-action",
                        message=f"*when header lacks goto or stop action: {block.header!r}.",
                        line=block.line,
                    )
                )
            ####
        ####
    ####
####


def summarize_issues(issues: list[ParseIssue]) -> dict[str, int]:
    counter = Counter(issue.severity for issue in issues)
    return {severity: counter.get(severity, 0) for severity in ("error", "warning", "note")}
####
