"""Line-oriented parser for CADAC ``input.asc`` cases."""

from __future__ import annotations

import math
import re
import shlex
from dataclasses import dataclass
from pathlib import Path

from .input_ast import (
    CadacDeckKind,
    CadacDeckReference,
    CadacEventBlock,
    CadacEventCondition,
    CadacInputCase,
    CadacModuleInvocation,
    CadacModuleStage,
    CadacMonteCarloSpec,
    CadacParameterAssignment,
    CadacRelationalOperator,
    CadacStochasticAssignment,
    CadacStochasticKind,
    CadacTimingSetting,
    CadacVehicleBlock,
    CadacVehicleEntry,
    source_name_for,
)

_INTEGER_PATTERN = re.compile(r"[+-]?\d+")


class CadacInputParseError(ValueError):
    """Source-located CADAC input parsing failure."""


####


@dataclass(frozen=True, slots=True)
class _SourceLine:
    number: int
    content: str
    comment: str | None


####


@dataclass(slots=True)
class _Cursor:
    lines: tuple[_SourceLine, ...]
    source_name: str
    index: int = 0

    def next(self, context: str) -> _SourceLine:
        if self.index >= len(self.lines):
            raise CadacInputParseError(f"{self.source_name}: unexpected end of file while reading {context}")
        ####
        line = self.lines[self.index]
        self.index += 1
        return line

    ####

    def remaining(self) -> tuple[_SourceLine, ...]:
        return self.lines[self.index :]

    ####


####


def parse_cadac_input(text: str, *, source_name: str = "<memory>") -> CadacInputCase:
    """Parse CADAC case structure, including sequential ``IF`` event blocks."""

    cursor = _Cursor(_content_lines(text), source_name)
    title_line = cursor.next("TITLE")
    title_tokens = _tokens(title_line, source_name)
    if len(title_tokens) < 2 or title_tokens[0].casefold() != "title":
        raise _error(source_name, title_line, "expected TITLE <source> [description]")
    ####
    title_source = title_tokens[1]
    description = " ".join(title_tokens[2:])

    monte_carlo: CadacMonteCarloSpec | None = None
    options_line = cursor.next("OPTIONS or MONTE")
    option_tokens = _tokens(options_line, source_name)
    if option_tokens and option_tokens[0].casefold() == "monte":
        if len(option_tokens) != 3:
            raise _error(source_name, options_line, "MONTE declaration must be MONTE <runs> <seed>")
        ####
        monte_carlo = CadacMonteCarloSpec(
            runs=_parse_positive_int(option_tokens[1], source_name, options_line),
            seed=_parse_int(option_tokens[2], source_name, options_line),
            source_line=options_line.number,
        )
        options_line = cursor.next("OPTIONS")
        option_tokens = _tokens(options_line, source_name)
    ####
    if not option_tokens or option_tokens[0].casefold() != "options":
        raise _error(source_name, options_line, "expected OPTIONS")
    ####
    options = tuple(option_tokens[1:])

    modules_header = cursor.next("MODULES")
    _expect_keyword(modules_header, "modules", source_name)
    modules = _parse_modules(cursor)

    timing_header = cursor.next("TIMING")
    _expect_keyword(timing_header, "timing", source_name)
    timing = _parse_timing(cursor)

    vehicles_header = cursor.next("VEHICLES")
    vehicle_header_tokens = _tokens(vehicles_header, source_name)
    if len(vehicle_header_tokens) != 2 or vehicle_header_tokens[0].casefold() != "vehicles":
        raise _error(source_name, vehicles_header, "expected VEHICLES <count>")
    ####
    vehicle_count = _parse_positive_int(vehicle_header_tokens[1], source_name, vehicles_header)
    vehicles = tuple(_parse_vehicle(cursor) for _ in range(vehicle_count))

    # Some CADAC families close the counted VEHICLES collection with a second
    # outer END after the final per-vehicle END.  Accept exactly that optional
    # structural terminator without making END generally ignorable.
    if cursor.remaining() and cursor.remaining()[0].content.casefold() == "end":
        cursor.next("VEHICLES outer END")
    ####

    end_time_line = cursor.next("ENDTIME")
    end_time_tokens = _tokens(end_time_line, source_name)
    if len(end_time_tokens) != 2 or end_time_tokens[0].casefold() != "endtime":
        raise _error(source_name, end_time_line, "expected ENDTIME <seconds>")
    ####
    end_time_s = _parse_finite_float(end_time_tokens[1], source_name, end_time_line)
    if end_time_s <= 0.0:
        raise _error(source_name, end_time_line, "ENDTIME must be positive")
    ####

    stop_line = cursor.next("STOP")
    _expect_keyword(stop_line, "stop", source_name)
    if cursor.remaining():
        line = cursor.remaining()[0]
        raise _error(source_name, line, "unexpected content after STOP")
    ####

    return CadacInputCase(
        source_name=source_name,
        title_source=title_source,
        description=description,
        monte_carlo=monte_carlo,
        options=options,
        modules=modules,
        timing=timing,
        vehicles=vehicles,
        end_time_s=end_time_s,
        stop_declared=True,
    )


####


def parse_cadac_input_file(path: str | Path) -> CadacInputCase:
    """Parse a UTF-8 CADAC input file with normalized path provenance."""

    source_path = Path(path)
    return parse_cadac_input(source_path.read_text(encoding="utf-8"), source_name=source_name_for(source_path))


####


def _parse_modules(cursor: _Cursor) -> tuple[CadacModuleInvocation, ...]:
    modules: list[CadacModuleInvocation] = []
    while True:
        line = cursor.next("MODULES body")
        if line.content.casefold() == "end":
            break
        ####
        tokens = _tokens(line, cursor.source_name)
        if len(tokens) != 2:
            raise _error(cursor.source_name, line, "module line must be <name> <stage,stage,...>")
        ####
        try:
            stages = tuple(CadacModuleStage(stage.casefold()) for stage in tokens[1].split(",") if stage)
        except ValueError as error:
            raise _error(cursor.source_name, line, f"unknown module stage in {tokens[1]!r}") from error
        ####
        modules.append(CadacModuleInvocation(name=tokens[0], stages=stages, source_line=line.number))
    ####
    if not modules:
        raise CadacInputParseError(f"{cursor.source_name}: MODULES block must not be empty")
    ####
    return tuple(modules)


####


def _parse_timing(cursor: _Cursor) -> tuple[CadacTimingSetting, ...]:
    settings: list[CadacTimingSetting] = []
    while True:
        line = cursor.next("TIMING body")
        if line.content.casefold() == "end":
            break
        ####
        tokens = _tokens(line, cursor.source_name)
        if len(tokens) != 2:
            raise _error(cursor.source_name, line, "timing line must be <name> <value>")
        ####
        settings.append(
            CadacTimingSetting(
                name=tokens[0],
                value=_parse_finite_float(tokens[1], cursor.source_name, line),
                source_line=line.number,
            )
        )
    ####
    if not settings:
        raise CadacInputParseError(f"{cursor.source_name}: TIMING block must not be empty")
    ####
    return tuple(settings)


####


def _parse_vehicle(cursor: _Cursor) -> CadacVehicleBlock:
    header = cursor.next("vehicle header")
    tokens = _tokens(header, cursor.source_name)
    if tokens and tokens[0].casefold() in {"endtime", "stop"}:
        raise _error(cursor.source_name, header, "declared VEHICLES count exceeds available vehicle blocks")
    ####
    if len(tokens) < 2:
        raise _error(cursor.source_name, header, "vehicle header must be <model> <role>")
    ####
    entries: list[CadacVehicleEntry] = []
    while True:
        line = cursor.next(f"vehicle {tokens[0]!r}")
        lowered = line.content.casefold()
        if lowered == "end":
            break
        ####
        if lowered.startswith("endtime ") or lowered == "stop":
            raise _error(cursor.source_name, line, f"vehicle {tokens[0]!r} is missing END")
        ####
        entry_tokens = _tokens(line, cursor.source_name)
        if not entry_tokens:
            continue
        ####
        if entry_tokens[0].casefold() == "if":
            entries.append(_parse_event(cursor, line, entry_tokens))
            continue
        ####
        if entry_tokens[0].casefold() == "endif":
            if len(entry_tokens) != 1:
                raise _error(cursor.source_name, line, "ENDIF must not contain additional tokens")
            ####
            # The historical C++ reader treats a standalone post-event ENDIF
            # as an unrecognized no-op. The published ADS6 autopilot case
            # contains one such directive, so retain that reader behavior
            # instead of rewriting caller-owned source data.
            continue
        ####
        entries.append(_parse_vehicle_scalar_or_deck(line, entry_tokens, cursor.source_name))
    ####
    return CadacVehicleBlock(
        model_name=tokens[0],
        role=" ".join(tokens[1:]),
        entries=tuple(entries),
        source_line=header.number,
    )


####


def _parse_event(cursor: _Cursor, if_line: _SourceLine, tokens: list[str]) -> CadacEventBlock:
    condition_tokens = _event_condition_tokens(tokens)
    if condition_tokens is None:
        raise _error(cursor.source_name, if_line, "event condition must be IF <variable> <operator> <scalar>")
    ####
    variable, operator_token, threshold_token = condition_tokens
    try:
        operator = CadacRelationalOperator(operator_token)
    except ValueError as error:
        raise _error(cursor.source_name, if_line, f"unsupported event operator {operator_token!r}") from error
    ####
    threshold = _parse_scalar(threshold_token)
    if not isinstance(threshold, (int, float)):
        raise _error(cursor.source_name, if_line, "event threshold must be numeric")
    ####
    assignments: list[CadacParameterAssignment] = []
    while True:
        line = cursor.next("event body")
        entry_tokens = _tokens(line, cursor.source_name)
        if entry_tokens and entry_tokens[0].casefold() == "endif":
            if len(entry_tokens) != 1:
                raise _error(cursor.source_name, line, "ENDIF must not contain additional tokens")
            ####
            return CadacEventBlock(
                condition=CadacEventCondition(variable=variable, operator=operator, value=threshold),
                assignments=tuple(assignments),
                source_line=if_line.number,
                end_source_line=line.number,
            )
        ####
        if not entry_tokens:
            continue
        ####
        if entry_tokens[0].casefold() == "if":
            raise _error(cursor.source_name, line, "nested CADAC event blocks are not supported")
        ####
        keyword = entry_tokens[0].upper()
        if keyword.endswith("_DECK") or keyword == "DECK":
            raise _error(cursor.source_name, line, "event-time deck replacement is not supported")
        ####
        if len(entry_tokens) != 2:
            raise _error(cursor.source_name, line, "event assignment must be <name> <scalar>")
        ####
        assignments.append(
            CadacParameterAssignment(
                name=entry_tokens[0],
                value=_parse_scalar(entry_tokens[1]),
                comment=line.comment,
                source_line=line.number,
            )
        )
    ####


def _event_condition_tokens(tokens: list[str]) -> tuple[str, str, str] | None:
    """Normalize spaced and legacy compact CADAC relational conditions."""

    if len(tokens) == 4 and tokens[0].casefold() == "if":
        return (tokens[1], tokens[2], tokens[3])
    ####
    if len(tokens) == 3 and tokens[0].casefold() == "if":
        compact = re.fullmatch(r"(<=|>=|=|<|>)(.+)", tokens[2])
        if compact is not None:
            return (tokens[1], compact.group(1), compact.group(2))
        ####
    ####
    if len(tokens) == 2 and tokens[0].casefold() == "if":
        compact = re.fullmatch(r"(.+?)(<=|>=|=|<|>)(.+)", tokens[1])
        if compact is not None:
            return (compact.group(1), compact.group(2), compact.group(3))
        ####
    ####
    return None
    ####


####


def _parse_vehicle_scalar_or_deck(
    line: _SourceLine,
    tokens: list[str],
    source_name: str,
) -> CadacVehicleEntry:
    keyword = tokens[0].upper()
    if keyword in {item.value for item in CadacStochasticKind}:
        return _parse_stochastic_assignment(line, tokens, source_name)
    ####
    if keyword.endswith("_DECK") or keyword == "DECK":
        if len(tokens) != 2:
            raise _error(source_name, line, "deck reference must be <kind> <path>")
        ####
        kind = CadacDeckKind(keyword) if keyword in {item.value for item in CadacDeckKind} else CadacDeckKind.GENERIC
        return CadacDeckReference(kind=kind, keyword=keyword, path=tokens[1], source_line=line.number)
    ####
    if len(tokens) != 2:
        raise _error(source_name, line, "vehicle assignment must be <name> <scalar>")
    ####
    return CadacParameterAssignment(
        name=tokens[0],
        value=_parse_scalar(tokens[1]),
        comment=line.comment,
        source_line=line.number,
    )


####


def _parse_stochastic_assignment(
    line: _SourceLine,
    tokens: list[str],
    source_name: str,
) -> CadacStochasticAssignment:
    kind = CadacStochasticKind(tokens[0].upper())
    expected_parameters = {
        CadacStochasticKind.GAUSSIAN: 2,
        CadacStochasticKind.MARKOV: 2,
        CadacStochasticKind.RAYLEIGH: 1,
    }[kind]
    if len(tokens) != expected_parameters + 2:
        raise _error(
            source_name,
            line,
            f"{kind.value} declaration requires a variable name and {expected_parameters} numeric parameter(s)",
        )
    ####
    parameters = tuple(_parse_finite_float(token, source_name, line) for token in tokens[2:])
    if kind is CadacStochasticKind.GAUSSIAN and parameters[1] < 0.0:
        raise _error(source_name, line, "GAUSS standard deviation must be nonnegative")
    ####
    if kind is CadacStochasticKind.MARKOV and (parameters[0] < 0.0 or parameters[1] <= 0.0):
        raise _error(source_name, line, "MARKOV sigma must be nonnegative and correlation time positive")
    ####
    if kind is CadacStochasticKind.RAYLEIGH and parameters[0] < 0.0:
        raise _error(source_name, line, "RAYL scale must be nonnegative")
    ####
    return CadacStochasticAssignment(
        kind=kind,
        name=tokens[1],
        parameters=parameters,
        comment=line.comment,
        source_line=line.number,
    )


####


def _content_lines(text: str) -> tuple[_SourceLine, ...]:
    lines: list[_SourceLine] = []
    for number, raw_line in enumerate(text.splitlines(), start=1):
        content, separator, comment = raw_line.partition("//")
        stripped = content.strip()
        if not stripped:
            continue
        ####
        lines.append(_SourceLine(number, stripped, comment.strip() if separator and comment.strip() else None))
    ####
    return tuple(lines)


####


def _tokens(line: _SourceLine, source_name: str) -> list[str]:
    try:
        return shlex.split(line.content, comments=False, posix=True)
    except ValueError as error:
        raise _error(source_name, line, str(error)) from error
    ####


####


def _parse_scalar(token: str) -> int | float | str:
    if _INTEGER_PATTERN.fullmatch(token):
        return int(token)
    ####
    try:
        value = float(token)
    except ValueError:
        return token
    ####
    return value


####


def _parse_int(token: str, source_name: str, line: _SourceLine) -> int:
    if not _INTEGER_PATTERN.fullmatch(token):
        raise _error(source_name, line, f"expected integer, received {token!r}")
    ####
    return int(token)


####


def _parse_positive_int(token: str, source_name: str, line: _SourceLine) -> int:
    if not _INTEGER_PATTERN.fullmatch(token):
        raise _error(source_name, line, f"expected positive integer, received {token!r}")
    ####
    value = int(token)
    if value <= 0:
        raise _error(source_name, line, "count must be positive")
    ####
    return value


####


def _parse_finite_float(token: str, source_name: str, line: _SourceLine) -> float:
    try:
        value = float(token)
    except ValueError as error:
        raise _error(source_name, line, f"expected numeric value, received {token!r}") from error
    ####
    if not math.isfinite(value):
        raise _error(source_name, line, "numeric values must be finite")
    ####
    return value


####


def _expect_keyword(line: _SourceLine, keyword: str, source_name: str) -> None:
    if line.content.casefold() != keyword.casefold():
        raise _error(source_name, line, f"expected {keyword.upper()}")
    ####


####


def _error(source_name: str, line: _SourceLine, message: str) -> CadacInputParseError:
    return CadacInputParseError(f"{source_name}:{line.number}: {message}")


####


__all__ = ["CadacInputParseError", "parse_cadac_input", "parse_cadac_input_file"]
