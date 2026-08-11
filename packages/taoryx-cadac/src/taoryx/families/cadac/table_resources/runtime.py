"""Runtime evaluation for canonical Taoryx rectilinear tables."""

from __future__ import annotations

import math
from itertools import product
from typing import Sequence

from .schema import TableBoundaryMode, TableResource


def lookup_table(table: TableResource, query: Sequence[float]) -> float:
    """Evaluate one canonical table using multilinear interpolation."""

    point = tuple(float(value) for value in query)
    if len(point) != table.dimension or any(not math.isfinite(value) for value in point):
        raise ValueError("query dimension must match the table and contain finite values")
    ####
    brackets = tuple(_bracket(axis.values, value, table.extrapolation.lower, table.extrapolation.upper) for axis, value in zip(table.axes, point, strict=True))
    sizes = tuple(len(axis.values) for axis in table.axes)
    strides = tuple(math.prod(sizes[index + 1 :]) for index in range(table.dimension))
    result = 0.0
    for corner in product((0, 1), repeat=table.dimension):
        weight = 1.0
        flat_index = 0
        for axis_index, (lower_index, fraction) in enumerate(brackets):
            axis = table.axes[axis_index].values
            selected = lower_index + corner[axis_index] if len(axis) > 1 else 0
            weight *= fraction if corner[axis_index] else 1.0 - fraction
            flat_index += selected * strides[axis_index]
        ####
        result += weight * table.values[flat_index]
    ####
    return result


####


def _bracket(
    axis: tuple[float, ...],
    query: float,
    lower_mode: TableBoundaryMode,
    upper_mode: TableBoundaryMode,
) -> tuple[int, float]:
    if len(axis) == 1:
        if query != axis[0] and (lower_mode is TableBoundaryMode.ERROR or upper_mode is TableBoundaryMode.ERROR):
            raise ValueError("query is outside a singleton table axis")
        ####
        return 0, 0.0
    ####
    if query < axis[0]:
        if lower_mode is TableBoundaryMode.ERROR:
            raise ValueError(f"query {query} is below table axis minimum {axis[0]}")
        ####
        if lower_mode is TableBoundaryMode.CLAMP:
            return 0, 0.0
        ####
        return 0, _fraction(axis, 0, query)
    ####
    if query > axis[-1]:
        if upper_mode is TableBoundaryMode.ERROR:
            raise ValueError(f"query {query} is above table axis maximum {axis[-1]}")
        ####
        if upper_mode is TableBoundaryMode.CLAMP:
            return len(axis) - 2, 1.0
        ####
        return len(axis) - 2, _fraction(axis, len(axis) - 2, query)
    ####
    for index, (left, right) in enumerate(zip(axis, axis[1:], strict=False)):
        if left <= query <= right:
            return index, _fraction(axis, index, query)
        ####
    ####
    raise RuntimeError("ascending axis did not produce an interpolation bracket")


####


def _fraction(axis: tuple[float, ...], index: int, query: float) -> float:
    return (query - axis[index]) / (axis[index + 1] - axis[index])


####


__all__ = ["lookup_table"]
