"""Renderer-independent inspection artifacts for TAOS table files.

The explorer boundary deliberately keeps source-faithful data beside prepared
runtime data.  A malformed or partial table can therefore still be inspected
without being mistaken for an executable table.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from enum import StrEnum
from itertools import product
from pathlib import Path
from typing import Any, cast

from taoryx.language.diagnostics import Diagnostic
from taoryx.language.models import TableCall, TableDefinition, TableDocument, TableOperation
from taoryx.language.semantic_validation import validate_table_file
from taoryx.language.table_parser import parse_table_file
from taoryx.tables import ExtrapolationMode, PreparedTable, prepare_table


class TableInspectionStatus(StrEnum):
    """What the inspection artifact can safely claim about a table."""

    PREPARED = "prepared"
    SOURCE_ONLY = "source-only"
    PARTIAL = "partial"
    INVALID = "invalid"
####


class TableInspectionFormat(StrEnum):
    """High-level representation selected from the parsed table structure."""

    REGULAR_GRID = "regular-grid"
    FULL_PROGRAM = "full-program"
####


@dataclass(frozen=True, slots=True)
class AxisInterpolationBracket:
    """One axis bracket used by a regular-grid interpolation query."""

    axis: str
    lower: float
    upper: float
    fraction: float
    status: str
    effective_query: float
####


@dataclass(frozen=True, slots=True)
class InterpolationExplanation:
    """Renderer-independent explanation of a regular-table query."""

    query: dict[str, float]
    effective_query: dict[str, float]
    brackets: tuple[AxisInterpolationBracket, ...]
    corners: tuple[dict[str, object], ...]
    value: float
    status: str

    def to_dict(self) -> dict[str, object]:
        return {
            "query": self.query,
            "effective_query": self.effective_query,
            "brackets": [
                {
                    "axis": item.axis,
                    "lower": item.lower,
                    "upper": item.upper,
                    "fraction": item.fraction,
                    "status": item.status,
                    "effective_query": item.effective_query,
                }
                for item in self.brackets
            ],
            "corners": list(self.corners),
            "value": self.value,
            "status": self.status,
        }
    ####
####


@dataclass(frozen=True, slots=True)
class TableInspection:
    """Inspection data for one declared table."""

    name: str
    table_type: str
    format: TableInspectionFormat
    status: TableInspectionStatus
    independent_variables: tuple[str, ...]
    shape: tuple[int, ...]
    options: dict[str, str | float]
    assignments: tuple[Any, ...]
    operations: tuple[TableOperation, ...]
    dependencies: tuple[str, ...]
    source_location: Any
    source_text: str | None
    diagnostics: tuple[Diagnostic, ...]
    prepared: PreparedTable | None = None
####

    @property
    def dimension(self) -> int:
        """Return the number of declared independent variables."""

        return len(self.independent_variables)
    ####

    def to_dict(self) -> dict[str, object]:
        """Return JSON-friendly metadata without discarding source records."""

        return {
            "name": self.name,
            "table_type": self.table_type,
            "format": self.format.value,
            "status": self.status.value,
            "independent_variables": list(self.independent_variables),
            "shape": list(self.shape),
            "options": dict(self.options),
            "dependencies": list(self.dependencies),
            "source_location": self.source_location.model_dump(),
            "source_text": self.source_text,
            "diagnostics": [diagnostic.model_dump(mode="json") for diagnostic in self.diagnostics],
            "assignments": [assignment.model_dump(mode="json") for assignment in self.assignments],
            "operations": [operation.model_dump(mode="json") for operation in self.operations],
            "prepared": None
            if self.prepared is None
            else {
                "axes": [list(axis) for axis in self.prepared.axes],
                "values": list(self.prepared.values),
                "strides": list(self.prepared.strides),
                "extrapolation": self.prepared.extrapolation.value,
            },
        }
    ####
####


@dataclass(frozen=True, slots=True)
class TableInspectionArtifact:
    """A complete, serializable inspection view of one table document."""

    path: str
    executable_complete: bool
    tables: tuple[TableInspection, ...]
    diagnostics: tuple[Diagnostic, ...]
    recovered_records: tuple[Any, ...]

    @property
    def valid(self) -> bool:
        """Whether the source has no error diagnostics."""

        return not any(diagnostic.severity.value == "error" for diagnostic in self.diagnostics)
    ####

    def catalog(self) -> list[dict[str, object]]:
        """Return compact catalog rows suitable for terminal or JSON output."""

        return [
            {
                "name": table.name,
                "type": table.table_type,
                "format": table.format.value,
                "shape": list(table.shape),
                "axes": list(table.independent_variables),
                "status": table.status.value,
                "dependencies": list(table.dependencies),
                "diagnostics": len(table.diagnostics),
            }
            for table in self.tables
        ]
    ####

    def to_dict(self) -> dict[str, object]:
        """Return the complete artifact as JSON-friendly data."""

        return {
            "path": self.path,
            "executable_complete": self.executable_complete,
            "valid": self.valid,
            "catalog": self.catalog(),
            "tables": [table.to_dict() for table in self.tables],
            "diagnostics": [diagnostic.model_dump(mode="json") for diagnostic in self.diagnostics],
            "recovered_records": [record.model_dump(mode="json") for record in self.recovered_records],
        }
    ####

    def format_catalog(self) -> str:
        """Format a compact human-readable table catalog."""

        lines = [f"{self.path}: {len(self.tables)} table(s)"]
        for row in self.catalog():
            axes = ",".join(str(axis) for axis in cast(list[object], row["axes"]))
            shape = "x".join(str(size) for size in cast(list[object], row["shape"])) or "-"
            lines.append(f"- {row['name']}: {row['type']} {row['format']} [{shape}] ({axes}) — {row['status']}")
        if self.diagnostics:
            lines.append(f"diagnostics: {len(self.diagnostics)}")
        return "\n".join(lines)
    ####

    def table(self, name: str | None = None) -> TableInspection:
        """Select one table, requiring a name for multi-table documents."""

        if not self.tables:
            raise ValueError("table document contains no tables")
        if name is None:
            if len(self.tables) != 1:
                raise ValueError("table name is required when the document contains multiple tables")
            return self.tables[0]
        for table in self.tables:
            if table.name.casefold() == name.casefold():
                return table
        raise KeyError(f"table {name!r} was not found")
    ####
####


def inspect_table_document(document: TableDocument, *, validate: bool = True) -> TableInspectionArtifact:
    """Build an inspection artifact from an already parsed table document."""

    diagnostics = list(validate_table_file(document)) if validate else list(document.diagnostics)
    by_table: dict[str, list[Diagnostic]] = {table.name.casefold(): [] for table in document.tables}
    for diagnostic in diagnostics:
        location = diagnostic.location
        for table in document.tables:
            if location is not None and table.location.line == location.line:
                by_table[table.name.casefold()].append(diagnostic)
                break
        ####
    ####
    inspections = tuple(_inspect_table(table, tuple(by_table.get(table.name.casefold(), ()))) for table in document.tables)
    return TableInspectionArtifact(
        path=document.tables[0].location.path if document.tables else "<memory>",
        executable_complete=document.executable_complete,
        tables=inspections,
        diagnostics=tuple(diagnostics),
        recovered_records=tuple(document.recovered_records),
    )
####


def inspect_table_file(path: str | Path) -> TableInspectionArtifact:
    """Parse and inspect a ``.tbl`` file while preserving diagnostics."""

    return inspect_table_document(parse_table_file(path))
####


def explain_interpolation(table: TableInspection, query: dict[str, float]) -> InterpolationExplanation:
    """Explain and evaluate one regular-grid query using runtime interpolation."""

    if table.prepared is None:
        raise ValueError(f"table {table.name!r} has no prepared regular-grid runtime")
    expected = {name.casefold() for name in table.independent_variables}
    provided = {name.casefold() for name in query}
    if expected != provided:
        raise ValueError(f"query axes mismatch; expected {sorted(expected)}, received {sorted(provided)}")
    normalized_query = {name.casefold(): float(value) for name, value in query.items()}
    brackets = tuple(
        _axis_bracket(name, table.prepared.axes[index], normalized_query[name.casefold()], table.prepared.extrapolation)
        for index, name in enumerate(table.independent_variables)
    )
    effective = {item.axis: item.effective_query for item in brackets}
    corners: list[dict[str, object]] = []
    for corner in product((0, 1), repeat=table.dimension):
        flat_index = 0
        weight = 1.0
        coordinate: dict[str, float] = {}
        for index, choice in enumerate(corner):
            bracket = brackets[index]
            selected = bracket.lower if choice == 0 else bracket.upper
            coordinate[bracket.axis] = selected
            weight *= bracket.fraction if choice else 1.0 - bracket.fraction
            flat_index += table.prepared.axes[index].index(selected) * table.prepared.strides[index]
        corners.append({"coordinate": coordinate, "flat_index": flat_index, "weight": weight, "value": table.prepared.values[flat_index]})
    from taoryx.tables import interpolate_nd

    statuses = {item.status for item in brackets}
    status = "clamped" if "clamped" in statuses else "extrapolated" if "extrapolated" in statuses else "interpolated"
    value = interpolate_nd(table.prepared, tuple(effective[name] for name in table.independent_variables))
    return InterpolationExplanation(normalized_query, effective, brackets, tuple(corners), value, status)
####


def _axis_bracket(name: str, axis: tuple[float, ...], query: float, extrapolation: ExtrapolationMode) -> AxisInterpolationBracket:
    ascending = axis[0] <= axis[-1]
    low, high = (axis[0], axis[-1]) if ascending else (axis[-1], axis[0])
    effective = query
    status = "inside"
    if query < low or query > high:
        if extrapolation is ExtrapolationMode.CLAMP:
            effective = low if query < low else high
            status = "clamped"
        else:
            status = "extrapolated"
    if ascending:
        lower_index = next((index for index in range(len(axis) - 1) if axis[index] <= effective <= axis[index + 1]), len(axis) - 2)
    else:
        lower_index = next((index for index in range(len(axis) - 1) if axis[index] >= effective >= axis[index + 1]), len(axis) - 2)
    lower, upper = axis[lower_index], axis[lower_index + 1]
    fraction = 0.0 if lower == upper else (effective - lower) / (upper - lower)
    return AxisInterpolationBracket(name, lower, upper, fraction, status, effective)
####


def _inspect_table(table: TableDefinition, diagnostics: tuple[Diagnostic, ...]) -> TableInspection:
    format_kind = TableInspectionFormat.REGULAR_GRID if table.format == "simple" else TableInspectionFormat.FULL_PROGRAM
    dependencies = tuple(sorted(_operation_dependencies(table.operations)))
    prepared: PreparedTable | None = None
    shape: tuple[int, ...] = ()
    status = TableInspectionStatus.SOURCE_ONLY
    if table.format == "simple":
        assignments = {assignment.name.casefold(): assignment for assignment in table.assignments}
        axes = [assignments[name.casefold()].values for name in table.independent_variables if name.casefold() in assignments]
        dependent = [assignment for assignment in table.assignments if assignment.name.casefold() not in {name.casefold() for name in table.independent_variables}]
        shape = tuple(len(axis) for axis in axes)
        if table.omissions:
            status = TableInspectionStatus.PARTIAL
        elif any(diagnostic.severity.value == "error" for diagnostic in diagnostics):
            status = TableInspectionStatus.INVALID
        elif len(axes) != len(table.independent_variables) or len(dependent) != 1:
            status = TableInspectionStatus.INVALID
        else:
            try:
                extrapolation = ExtrapolationMode(str(table.options.get("extrapolation", "extrap")))
                prepared = prepare_table(axes, dependent[0].values, extrapolation=extrapolation)
                status = TableInspectionStatus.PREPARED
            except (TypeError, ValueError):
                status = TableInspectionStatus.INVALID
            ####
        ####
    ####
    return TableInspection(
        name=table.name,
        table_type=table.table_type,
        format=format_kind,
        status=status,
        independent_variables=tuple(table.independent_variables),
        shape=shape,
        options=dict(table.options),
        assignments=tuple(table.assignments),
        operations=tuple(table.operations),
        dependencies=dependencies,
        source_location=table.location,
        source_text=table.source_text,
        diagnostics=diagnostics,
        prepared=prepared,
    )
####


def _operation_dependencies(operations: Iterable[TableOperation]) -> set[str]:
    dependencies: set[str] = set()
    for operation in operations:
        if isinstance(operation.operand, TableCall):
            dependencies.add(operation.operand.name)
        if operation.nested is not None:
            dependencies.update(_operation_dependencies((operation.nested,)))
        ####
    ####
    return dependencies
####
