"""Typed table preparation and multilinear interpolation contracts."""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping, MutableMapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from itertools import product

from .language.models import TableAssignment, TableCall, TableOperation


class ExtrapolationMode(StrEnum):
    """Behavior outside an axis domain."""

    LINEAR = "extrap"
    CLAMP = "no-extrap"
####


@dataclass(frozen=True, slots=True)
class PreparedTable:
    """Validated row-major square table with explicit axis order."""

    axes: tuple[tuple[float, ...], ...]
    values: tuple[float, ...]
    strides: tuple[int, ...]
    extrapolation: ExtrapolationMode

    @property
    def dimension(self) -> int:
        return len(self.axes)
    ####
####


@dataclass(slots=True)
class TableEvaluationContext:
    """Values visible while executing a full-table operation program."""

    values: Mapping[str, float]
    tables: Mapping[str, PreparedTable]
    storage: dict[str, float]
    evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] = field(default_factory=dict)

    @classmethod
    def from_values(
        cls,
        values: Mapping[str, float],
        tables: Mapping[str, PreparedTable] | None = None,
        evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] | None = None,
    ) -> "TableEvaluationContext":
        return cls(values, tables or {}, {}, evaluators or {})
####


@dataclass(frozen=True, slots=True)
class FullTableResult:
    """Value and storage state returned by a full-table evaluation."""

    value: float
    storage: tuple[tuple[str, float], ...]
####


@dataclass(frozen=True, slots=True)
class SkewedTableSlice:
    """One outer-grid slice of a skewed table."""

    outer_coordinates: tuple[float, ...]
    axis: tuple[float, ...]
    values: tuple[float, ...]
####


@dataclass(frozen=True, slots=True)
class PreparedSkewedTable:
    """Validated grouped representation for Chapter 3 skewed tables."""

    slices: tuple[SkewedTableSlice, ...]
    outer_axes: tuple[tuple[float, ...], ...]
    extrapolation: ExtrapolationMode
####


def prepare_table(
    axes: Sequence[Sequence[float]],
    values: Sequence[float],
    *,
    extrapolation: ExtrapolationMode = ExtrapolationMode.LINEAR,
) -> PreparedTable:
    """Evaluate TAOS-ALG-TABLE-001 and prepare a TABLE-002 runtime object."""

    normalized_axes = tuple(tuple(float(value) for value in axis) for axis in axes)
    if not normalized_axes or len(normalized_axes) > 5:
        raise ValueError("tables require between one and five independent axes")
    for axis in normalized_axes:
        if len(axis) < 1 or any(not math.isfinite(value) for value in axis):
            raise ValueError("table axes must contain finite values")
        if len(axis) > 1 and not (all(left < right for left, right in zip(axis, axis[1:], strict=False)) or all(left > right for left, right in zip(axis, axis[1:], strict=False))):
            raise ValueError("table axes must be strictly monotonic")
        ####
    ####
    normalized_values = tuple(float(value) for value in values)
    if not normalized_values or any(not math.isfinite(value) for value in normalized_values):
        raise ValueError("table values must be finite and non-empty")
    expected = math.prod(len(axis) for axis in normalized_axes)
    if len(normalized_values) != expected:
        raise ValueError(f"table requires {expected} values for its axis sizes")
    strides: list[int] = []
    for index in range(len(normalized_axes)):
        strides.append(math.prod(len(axis) for axis in normalized_axes[index + 1 :]))
    if not isinstance(extrapolation, ExtrapolationMode):
        raise ValueError("unsupported table extrapolation mode")
    return PreparedTable(normalized_axes, normalized_values, tuple(strides), extrapolation)
####


def interpolate_nd(table: PreparedTable, query: Sequence[float]) -> float:
    """Evaluate TAOS-ALG-TABLE-002 using row-major multilinear interpolation."""

    point = tuple(float(value) for value in query)
    if len(point) != table.dimension or any(not math.isfinite(value) for value in point):
        raise ValueError("query dimension must match the table and contain finite values")
    brackets = tuple(_bracket(axis, value, table.extrapolation) for axis, value in zip(table.axes, point, strict=True))
    result = 0.0
    for corner in product((0, 1), repeat=table.dimension):
        weight = 1.0
        flat_index = 0
        for axis_index, (lower_index, fraction) in enumerate(brackets):
            selected = lower_index + corner[axis_index] if len(table.axes[axis_index]) > 1 else 0
            weight *= fraction if corner[axis_index] else 1.0 - fraction
            flat_index += selected * table.strides[axis_index]
        result += weight * table.values[flat_index]
    return result
####


def evaluate_full_table(
    operations: Sequence[TableOperation],
    context: TableEvaluationContext,
    *,
    max_steps: int = 10000,
) -> FullTableResult:
    """Evaluate TAOS-ALG-TABLE-003 using an accumulator and control flow."""

    if not operations:
        raise ValueError("full table requires at least one operation")
    if max_steps <= 0:
        raise ValueError("max_steps must be positive")
    labels = {operation.label.casefold(): index for index, operation in enumerate(operations) if operation.label}
    accumulator = 0.0
    index = 0
    steps = 0
    while index < len(operations):
        if steps >= max_steps:
            raise RuntimeError("full-table control flow exceeded max_steps")
        operation = operations[index]
        steps += 1
        operator = operation.operator.casefold()
        if operator == "end":
            break
        if operator == "goto":
            if not isinstance(operation.operand, str) or operation.operand.casefold() not in labels:
                raise ValueError("goto target is undefined")
            index = labels[operation.operand.casefold()]
            continue
        if operator == "if":
            if operation.condition is None:
                raise ValueError("if operation requires a condition")
            if _evaluate_condition(operation.condition, context, accumulator) and operation.nested is not None:
                nested = operation.nested
                if nested.operator.casefold() == "goto":
                    if not isinstance(nested.operand, str) or nested.operand.casefold() not in labels:
                        raise ValueError("goto target is undefined")
                    index = labels[nested.operand.casefold()]
                    continue
                accumulator = _apply_operation(nested, accumulator, context)
            index += 1
            continue
        accumulator = _apply_operation(operation, accumulator, context)
        index += 1
    else:
        raise ValueError("full table must terminate with an end operation")
    return FullTableResult(accumulator, tuple(sorted(context.storage.items())))
####


def clear_and_store(storage: MutableMapping[str, float], name: str, value: float) -> float:
    """Store one full-table value and return the cleared accumulator."""

    key = name.casefold().strip()
    if not key:
        raise ValueError("storage variable name must not be empty")
    if key in storage:
        raise ValueError(f"duplicate storage variable: {name}")
    stored = float(value)
    if not math.isfinite(stored):
        raise ValueError("stored table values must be finite")
    storage[key] = stored
    return 0.0
####


def resolve_table_operand(
    operand: str | float | TableCall,
    context: TableEvaluationContext,
    *,
    assignments: Sequence[TableAssignment] = (),
) -> float:
    """Evaluate TAOS-ALG-TABLE-004 operands in the current context."""

    if isinstance(operand, (int, float)):
        return float(operand)
    if isinstance(operand, TableCall):
        evaluator = context.evaluators.get(operand.name.casefold())
        query = tuple(_resolve_name(argument, context) for argument in operand.arguments)
        if evaluator is not None:
            evaluate_call = getattr(evaluator, "evaluate_call", None)
            if evaluate_call is not None:
                return float(evaluate_call(context.values, query))
            if query:
                raise ValueError(f"table {operand.name!r} does not support lookup arguments")
            return float(evaluator(context.values))
        table = context.tables.get(operand.name.casefold())
        if table is None:
            table = _inline_table(operand, assignments)
        return interpolate_nd(table, query)
    evaluator = context.evaluators.get(operand.casefold())
    if evaluator is not None:
        return float(evaluator(context.values))
    return _resolve_name(operand, context)
####


def apply_table_operation(
    accumulator: float,
    operator: str,
    operand: float | None = None,
) -> float:
    """Evaluate TAOS-ALG-TABLE-005's arithmetic operation dispatch."""

    name = operator.casefold()
    if name == "add":
        return accumulator + _require_operand(operand, name)
    if name == "sub":
        return accumulator - _require_operand(operand, name)
    if name == "mult":
        return accumulator * _require_operand(operand, name)
    if name == "div":
        divisor = _require_operand(operand, name)
        if divisor == 0.0:
            raise ZeroDivisionError("full-table div operand is zero")
        return accumulator / divisor
    if name == "idiv":
        if accumulator == 0.0:
            raise ZeroDivisionError("full-table idiv accumulator is zero")
        return _require_operand(operand, name) / accumulator
    if name == "exp":
        return accumulator ** _require_operand(operand, name)
    if name == "iexp":
        return _require_operand(operand, name) ** accumulator
    if name == "max":
        return max(accumulator, _require_operand(operand, name))
    if name == "min":
        return min(accumulator, _require_operand(operand, name))
    if name == "set":
        return _require_operand(operand, name)
    if name == "abs":
        return abs(accumulator)
    if name == "neg":
        return -accumulator
    if name == "sqr":
        return accumulator * accumulator
    if name == "sqrt":
        return math.sqrt(accumulator)
    if name == "ln":
        return math.log(accumulator)
    if name == "log":
        return math.log10(accumulator)
    if name == "e":
        return math.exp(accumulator)
    if name == "sin":
        return math.sin(math.radians(accumulator))
    if name == "cos":
        return math.cos(math.radians(accumulator))
    if name == "tan":
        return math.tan(math.radians(accumulator))
    if name == "asin":
        return math.degrees(math.asin(accumulator))
    if name == "acos":
        return math.degrees(math.acos(accumulator))
    if name == "atan":
        return math.degrees(math.atan(accumulator))
    if name == "zero":
        return 0.0
    raise ValueError(f"unsupported full-table operation: {operator}")
####


def accumulate_table_values(values: Sequence[float]) -> float:
    """Evaluate TAOS-ALG-TABLE-009 by summing active table values."""

    normalized = tuple(float(value) for value in values)
    if any(not math.isfinite(value) for value in normalized):
        raise ValueError("table values must be finite")
    return math.fsum(normalized)
####


def prepare_skewed_table(
    slices: Sequence[SkewedTableSlice],
    *,
    extrapolation: ExtrapolationMode = ExtrapolationMode.LINEAR,
) -> PreparedSkewedTable:
    """Prepare grouped skewed data for TAOS-ALG-TABLE-008."""

    normalized = tuple(
        SkewedTableSlice(
            tuple(float(value) for value in item.outer_coordinates),
            tuple(float(value) for value in item.axis),
            tuple(float(value) for value in item.values),
        )
        for item in slices
    )
    if not normalized:
        raise ValueError("skewed table requires at least one slice")
    outer_dimension = len(normalized[0].outer_coordinates)
    if outer_dimension < 1:
        raise ValueError("skewed table slices require at least one outer coordinate")
    if len({item.outer_coordinates for item in normalized}) != len(normalized):
        raise ValueError("skewed table outer coordinates must be unique")
    for item in normalized:
        if len(item.outer_coordinates) != outer_dimension:
            raise ValueError("skewed table outer dimensions must match")
        if len(item.axis) != len(item.values) or len(item.axis) < 1:
            raise ValueError("skewed table slice axis and values must have equal nonzero length")
        if any(not math.isfinite(value) for value in item.outer_coordinates + item.axis + item.values):
            raise ValueError("skewed table values must be finite")
        if len(item.axis) > 1 and not all(left < right for left, right in zip(item.axis, item.axis[1:], strict=False)):
            raise ValueError("skewed table slice axes must be strictly ascending")
        ####
    ####
    outer_axes = tuple(tuple(sorted({item.outer_coordinates[index] for item in normalized})) for index in range(outer_dimension))
    if not isinstance(extrapolation, ExtrapolationMode):
        raise ValueError("unsupported table extrapolation mode")
    return PreparedSkewedTable(normalized, outer_axes, extrapolation)
####


def interpolate_skewed(table: PreparedSkewedTable, query: Sequence[float]) -> float:
    """Interpolate TAOS-ALG-TABLE-008 grouped skewed tabulated data."""

    point = tuple(float(value) for value in query)
    if len(point) != len(table.outer_axes) + 1 or any(not math.isfinite(value) for value in point):
        raise ValueError("skewed query dimension must match the table")
    def evaluate_level(slices: tuple[SkewedTableSlice, ...], level: int) -> float:
        if level == len(table.outer_axes):
            if len(slices) != 1:
                raise ValueError("skewed table contains duplicate nested slices")
            item = slices[0]
            inner = prepare_table((item.axis,), item.values, extrapolation=table.extrapolation)
            return interpolate_nd(inner, (point[-1],))
        grouped: dict[float, list[SkewedTableSlice]] = {}
        for item in slices:
            grouped.setdefault(item.outer_coordinates[level], []).append(item)
        axis = tuple(sorted(grouped))
        values = tuple(evaluate_level(tuple(grouped[coordinate]), level + 1) for coordinate in axis)
        outer = prepare_table((axis,), values, extrapolation=table.extrapolation)
        return interpolate_nd(outer, (point[level],))
    ####

    return evaluate_level(table.slices, 0)
####


def _apply_operation(operation: TableOperation, accumulator: float, context: TableEvaluationContext) -> float:
    operator = operation.operator.casefold()
    if operator == "csto":
        if not isinstance(operation.operand, str):
            raise ValueError("csto requires a storage name")
        return clear_and_store(context.storage, operation.operand, accumulator)
    operand = None if operation.operand is None else resolve_table_operand(operation.operand, context, assignments=operation.assignments)
    return apply_table_operation(accumulator, operator, operand)
####


def _resolve_name(name: str, context: TableEvaluationContext) -> float:
    key = name.casefold()
    if key in context.storage:
        return context.storage[key]
    for source_name, value in context.values.items():
        if source_name.casefold() == key:
            return float(value)
    try:
        return float(name)
    except ValueError:
        pass
    raise KeyError(f"undefined table operand: {name}")
####


def _require_operand(operand: float | None, operator: str) -> float:
    if operand is None:
        raise ValueError(f"{operator} requires an operand")
    return operand
####


def _evaluate_condition(condition: str, context: TableEvaluationContext, accumulator: float) -> bool:
    match = re.fullmatch(r"\s*(\S+)\s*(<=|>=|==|!=|<|>)\s*(\S+)\s*", condition)
    if match is None:
        raise ValueError(f"unsupported table condition: {condition}")
    left = accumulator if match.group(1).casefold() in {"value", "table"} else _resolve_name(match.group(1), context)
    right = float(match.group(3)) if _is_number(match.group(3)) else _resolve_name(match.group(3), context)
    return {
        "<": left < right,
        "<=": left <= right,
        "==": left == right,
        "!=": left != right,
        ">=": left >= right,
        ">": left > right,
    }[match.group(2)]
####


def _is_number(value: str) -> bool:
    try:
        float(value)
    except ValueError:
        return False
    return True
####


def _outer_grid(axes: tuple[tuple[float, ...], ...]) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(values) for values in product(*axes))
####


def _inline_table(operand: TableCall, assignments: Sequence[TableAssignment]) -> PreparedTable:
    rows = tuple(assignments)
    names = tuple(operand.arguments) + (operand.name,)
    by_name = {row.name.casefold(): tuple(float(value) for value in row.values) for row in rows}
    missing = [name for name in names if name.casefold() not in by_name]
    if missing:
        raise ValueError(f"inline table is missing assignments: {missing}")
    return prepare_table(tuple(by_name[name.casefold()] for name in operand.arguments), by_name[operand.name.casefold()])
####


def _bracket(axis: tuple[float, ...], query: float, mode: ExtrapolationMode) -> tuple[int, float]:
    if len(axis) == 1:
        return 0, 0.0
    ascending = axis[0] < axis[-1]
    outside = query < axis[0] or query > axis[-1] if ascending else query > axis[0] or query < axis[-1]
    if outside and mode is ExtrapolationMode.CLAMP:
        if ascending:
            query = axis[0] if query < axis[0] else axis[-1]
        else:
            query = axis[0] if query > axis[0] else axis[-1]
    if ascending:
        index = next((i for i in range(len(axis) - 1) if axis[i] <= query <= axis[i + 1]), len(axis) - 2)
    else:
        index = next((i for i in range(len(axis) - 1) if axis[i] >= query >= axis[i + 1]), len(axis) - 2)
    fraction = (query - axis[index]) / (axis[index + 1] - axis[index])
    return index, fraction
####
