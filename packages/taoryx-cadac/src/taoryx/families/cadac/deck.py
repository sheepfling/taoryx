"""CADAC deck intake with source-compatible boundary behavior."""

from __future__ import annotations

import math
import re
from enum import StrEnum
from itertools import product
from pathlib import Path
from typing import Sequence

from pydantic import Field, computed_field, model_validator

from .input_ast import CadacModel, source_name_for

_TABLE_HEADER = re.compile(r"(?P<dimension>\d+)DIM\s+(?P<name>\S+)", re.IGNORECASE)
_AXIS_SIZE = re.compile(r"NX(?P<axis>\d+)\s+(?P<size>\d+)", re.IGNORECASE)
_NUMERIC_TOKEN = re.compile(r"[+-]?(?:(?:\d+(?:\.\d*)?)|(?:\.\d+))(?:[Ee][+-]?\d+)?")


class CadacDeckParseError(ValueError):
    """Source-located CADAC deck parsing failure."""


####


class CadacBoundaryMode(StrEnum):
    """One table-domain boundary policy."""

    LINEAR = "linear"
    CLAMP = "clamp"


####


class CadacTablePolicy(CadacModel):
    """Numerical lower/upper-domain behavior used by CADAC tables."""

    lower: CadacBoundaryMode
    upper: CadacBoundaryMode


####


CADAC_TABLE_POLICY = CadacTablePolicy(lower=CadacBoundaryMode.LINEAR, upper=CadacBoundaryMode.CLAMP)


class CadacDeckTable(CadacModel):
    """One immutable row-major 1-D, 2-D, or 3-D CADAC table."""

    name: str = Field(min_length=1)
    dimension: int = Field(ge=1, le=3)
    axis_sizes: tuple[int, ...]
    axes: tuple[tuple[float, ...], ...]
    values: tuple[float, ...]
    description: str | None = None
    source_line: int = Field(ge=1)

    @computed_field
    @property
    def strides(self) -> tuple[int, ...]:
        return tuple(math.prod(self.axis_sizes[index + 1 :]) for index in range(self.dimension))

    ####

    def interpolate(
        self,
        query: Sequence[float],
        *,
        policy: CadacTablePolicy = CADAC_TABLE_POLICY,
    ) -> float:
        """Evaluate the table with multilinear interpolation."""

        return interpolate_cadac_table(self, query, policy=policy)

    ####

    @model_validator(mode="after")
    def validate_shape(self) -> "CadacDeckTable":
        if len(self.axis_sizes) != self.dimension or len(self.axes) != self.dimension:
            raise ValueError("dimension, axis_sizes, and axes must agree")
        ####
        for axis_index, (declared_size, axis) in enumerate(zip(self.axis_sizes, self.axes, strict=True), start=1):
            if declared_size <= 0 or len(axis) != declared_size:
                raise ValueError(f"axis {axis_index} size does not match its declaration")
            ####
            if any(not math.isfinite(value) for value in axis):
                raise ValueError(f"axis {axis_index} contains non-finite values")
            ####
            if len(axis) > 1:
                ascending = all(left < right for left, right in zip(axis, axis[1:], strict=False))
                descending = all(left > right for left, right in zip(axis, axis[1:], strict=False))
                if not (ascending or descending):
                    raise ValueError(f"axis {axis_index} must be strictly monotonic")
                ####
            ####
        ####
        expected = math.prod(self.axis_sizes)
        if len(self.values) != expected:
            raise ValueError(f"table requires {expected} values, received {len(self.values)}")
        ####
        if any(not math.isfinite(value) for value in self.values):
            raise ValueError("table values must be finite")
        ####
        return self

    ####


####


class CadacDeck(CadacModel):
    """One parsed CADAC table deck."""

    source_name: str = Field(min_length=1)
    title: str = ""
    tables: tuple[CadacDeckTable, ...] = Field(min_length=1)

    def table(self, name: str) -> CadacDeckTable:
        """Return one case-insensitive table by name."""

        key = name.casefold()
        for table in self.tables:
            if table.name.casefold() == key:
                return table
            ####
        ####
        raise KeyError(name)

    ####

    @model_validator(mode="after")
    def validate_unique_tables(self) -> "CadacDeck":
        names = [table.name.casefold() for table in self.tables]
        if len(names) != len(set(names)):
            raise ValueError("deck contains duplicate table names")
        ####
        return self

    ####


####


def parse_cadac_deck(text: str, *, source_name: str = "<memory>") -> CadacDeck:
    """Parse CADAC 1-D/2-D/3-D table decks using the source row layout.

    Rectangular tables carry each independent axis in the leading columns
    while X1 rows carry the full product of the remaining dimensions as
    row-major data. This mirrors the CADAC ``read_tables`` implementation.
    """

    raw_lines = text.splitlines()
    title = ""
    tables: list[CadacDeckTable] = []
    index = 0
    while index < len(raw_lines):
        content, comment = _split_line(raw_lines[index])
        line_number = index + 1
        if not content:
            index += 1
            continue
        ####
        if content.casefold().startswith("title "):
            title = content.split(maxsplit=1)[1]
            index += 1
            continue
        ####
        match = _TABLE_HEADER.fullmatch(content)
        if match is None:
            if re.match(r"\d+DIM\b", content, flags=re.IGNORECASE):
                raise CadacDeckParseError(f"{source_name}:{line_number}: malformed table declaration {content!r}")
            ####
            index += 1
            continue
        ####
        dimension = int(match.group("dimension"))
        if dimension not in {1, 2, 3}:
            raise CadacDeckParseError(f"{source_name}:{line_number}: {dimension}DIM tables are not supported by this prototype")
        ####
        name = match.group("name")
        index, dimension_content, dimension_comment, dimension_line = _next_content_line(raw_lines, index + 1, source_name)
        size_pairs = [(int(item.group("axis")), int(item.group("size"))) for item in _AXIS_SIZE.finditer(dimension_content)]
        expected_axes = tuple(range(1, dimension + 1))
        if tuple(axis for axis, _ in size_pairs) != expected_axes:
            raise CadacDeckParseError(f"{source_name}:{dimension_line}: expected " + " ".join(f"NX{axis} <size>" for axis in expected_axes))
        ####
        axis_sizes = tuple(size for _, size in size_pairs)
        index += 1
        if dimension == 1:
            index, axes, values = _parse_1d_rows(raw_lines, index, axis_sizes[0], source_name)
            axis_sizes = (len(axes[0]),)
        else:
            index, axes, values = _parse_multidimensional_rows(raw_lines, index, axis_sizes, source_name)
        ####
        tables.append(
            CadacDeckTable(
                name=name,
                dimension=dimension,
                axis_sizes=axis_sizes,
                axes=axes,
                values=values,
                description=dimension_comment or comment,
                source_line=line_number,
            )
        )
    ####
    if not tables:
        raise CadacDeckParseError(f"{source_name}: no supported table declarations were found")
    ####
    return CadacDeck(source_name=source_name, title=title, tables=tuple(tables))


####


def parse_cadac_deck_file(path: str | Path) -> CadacDeck:
    """Parse a UTF-8 CADAC deck file with normalized path provenance."""

    source_path = Path(path)
    return parse_cadac_deck(source_path.read_text(encoding="utf-8"), source_name=source_name_for(source_path))


####


def interpolate_cadac_table(
    table: CadacDeckTable,
    query: Sequence[float],
    *,
    policy: CadacTablePolicy = CADAC_TABLE_POLICY,
) -> float:
    """Evaluate a prepared table with per-side CADAC extrapolation policy."""

    point = tuple(float(value) for value in query)
    if len(point) != table.dimension or any(not math.isfinite(value) for value in point):
        raise ValueError("query dimension must match the table and contain finite values")
    ####
    brackets = tuple(_bracket(axis, value, policy) for axis, value in zip(table.axes, point, strict=True))
    result = 0.0
    for corner in product((0, 1), repeat=table.dimension):
        weight = 1.0
        flat_index = 0
        for axis_index, (lower_index, fraction) in enumerate(brackets):
            axis = table.axes[axis_index]
            selected = lower_index + corner[axis_index] if len(axis) > 1 else 0
            weight *= fraction if corner[axis_index] else 1.0 - fraction
            flat_index += selected * table.strides[axis_index]
        ####
        result += weight * table.values[flat_index]
    ####
    return result


####


def _parse_1d_rows(
    raw_lines: list[str],
    index: int,
    size: int,
    source_name: str,
) -> tuple[int, tuple[tuple[float, ...], ...], tuple[float, ...]]:
    axis: list[float] = []
    values: list[float] = []
    for _ in range(size):
        index, content, _, line_number = _next_content_line(raw_lines, index, source_name)
        numbers = _numeric_tokens(content, source_name, line_number)
        if len(numbers) != 2:
            raise CadacDeckParseError(f"{source_name}:{line_number}: 1DIM row must contain axis and value")
        ####
        axis_value = numbers[0]
        table_value = numbers[1]
        if axis and math.isclose(axis_value, axis[-1], rel_tol=1.0e-12, abs_tol=1.0e-12):
            if not math.isclose(table_value, values[-1], rel_tol=1.0e-12, abs_tol=1.0e-12):
                raise CadacDeckParseError(f"{source_name}:{line_number}: duplicate 1DIM axis value has conflicting table data")
            ####
        else:
            axis.append(axis_value)
            values.append(table_value)
        ####
        index += 1
    ####
    return index, (tuple(axis),), tuple(values)


####


def _parse_multidimensional_rows(
    raw_lines: list[str],
    index: int,
    axis_sizes: tuple[int, ...],
    source_name: str,
) -> tuple[int, tuple[tuple[float, ...], ...], tuple[float, ...]]:
    """Parse CADAC's rectangular 2-D/3-D source row layout."""

    axes: list[list[float]] = [[] for _ in axis_sizes]
    values: list[float] = []
    row_data_width = math.prod(axis_sizes[1:])
    for row_index in range(max(axis_sizes)):
        index, content, _, line_number = _next_content_line(raw_lines, index, source_name)
        numbers = _numeric_tokens(content, source_name, line_number)
        axis_field_count = sum(row_index < size for size in axis_sizes)
        data_field_count = row_data_width if row_index < axis_sizes[0] else 0
        expected = axis_field_count + data_field_count
        if len(numbers) != expected:
            raise CadacDeckParseError(
                f"{source_name}:{line_number}: {len(axis_sizes)}DIM row {row_index + 1} requires {expected} numeric fields, received {len(numbers)}"
            )
        ####
        offset = 0
        for axis_index, size in enumerate(axis_sizes):
            if row_index < size:
                axes[axis_index].append(numbers[offset])
                offset += 1
            ####
        ####
        if row_index < axis_sizes[0]:
            values.extend(numbers[offset:])
        ####
        index += 1
    ####
    return index, tuple(tuple(axis) for axis in axes), tuple(values)


####


def _bracket(axis: tuple[float, ...], query: float, policy: CadacTablePolicy) -> tuple[int, float]:
    if len(axis) == 1:
        return 0, 0.0
    ####
    ascending = axis[0] < axis[-1]
    minimum = axis[0] if ascending else axis[-1]
    maximum = axis[-1] if ascending else axis[0]
    if query < minimum:
        if policy.lower is CadacBoundaryMode.CLAMP:
            return (0, 0.0) if ascending else (len(axis) - 2, 1.0)
        ####
        index = 0 if ascending else len(axis) - 2
        return index, _fraction(axis, index, query)
    ####
    if query > maximum:
        if policy.upper is CadacBoundaryMode.CLAMP:
            return (len(axis) - 2, 1.0) if ascending else (0, 0.0)
        ####
        index = len(axis) - 2 if ascending else 0
        return index, _fraction(axis, index, query)
    ####
    for index, (left, right) in enumerate(zip(axis, axis[1:], strict=False)):
        if min(left, right) <= query <= max(left, right):
            return index, _fraction(axis, index, query)
        ####
    ####
    raise RuntimeError("monotonic axis did not produce an interpolation bracket")


####


def _fraction(axis: tuple[float, ...], index: int, query: float) -> float:
    return (query - axis[index]) / (axis[index + 1] - axis[index])


####


def _next_content_line(
    raw_lines: list[str],
    index: int,
    source_name: str,
) -> tuple[int, str, str | None, int]:
    while index < len(raw_lines):
        content, comment = _split_line(raw_lines[index])
        if content:
            return index, content, comment, index + 1
        ####
        index += 1
    ####
    raise CadacDeckParseError(f"{source_name}: unexpected end of file while reading table data")


####


def _split_line(raw_line: str) -> tuple[str, str | None]:
    content, separator, comment = raw_line.partition("//")
    normalized_comment = comment.strip() if separator and comment.strip() else None
    return content.strip(), normalized_comment


####


def _numeric_tokens(content: str, source_name: str, line_number: int) -> tuple[float, ...]:
    """Read source floats, including legacy adjacent signed table values."""

    values: list[float] = []
    position = 0
    while position < len(content):
        if content[position].isspace():
            position += 1
            continue
        ####
        match = _NUMERIC_TOKEN.match(content, position)
        if match is None:
            token = content[position:].split(maxsplit=1)[0]
            raise CadacDeckParseError(f"{source_name}:{line_number}: non-numeric table token {token!r}")
        ####
        token = match.group(0)
        try:
            value = float(token)
        except ValueError as error:
            raise CadacDeckParseError(f"{source_name}:{line_number}: non-numeric table token {token!r}") from error
        ####
        if not math.isfinite(value):
            raise CadacDeckParseError(f"{source_name}:{line_number}: table values must be finite")
        ####
        values.append(value)
        position = match.end()
    ####
    return tuple(values)


####


__all__ = [
    "CADAC_TABLE_POLICY",
    "CadacBoundaryMode",
    "CadacDeck",
    "CadacDeckParseError",
    "CadacDeckTable",
    "CadacTablePolicy",
    "interpolate_cadac_table",
    "parse_cadac_deck",
    "parse_cadac_deck_file",
]
