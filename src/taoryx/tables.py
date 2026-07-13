"""Typed table preparation and multilinear interpolation contracts."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from itertools import product


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
