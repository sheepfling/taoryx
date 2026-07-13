"""Numerical primitives for cataloged optimization and search algorithms."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import StrEnum

from .equations import Matrix, guidance_newton_update


class DifferenceMode(StrEnum):
    """Finite-difference stencil selected by the caller."""

    FORWARD = "forward"
    CENTRAL = "central"
####


@dataclass(frozen=True, slots=True)
class Jacobian:
    """Immutable row-major Jacobian with explicit output/input dimensions."""

    rows: tuple[tuple[float, ...], ...]

    def __post_init__(self) -> None:
        width = len(self.rows[0]) if self.rows else 0
        if any(len(row) != width for row in self.rows):
            raise ValueError("Jacobian rows must have equal width")
        if any(not math.isfinite(value) for row in self.rows for value in row):
            raise ValueError("Jacobian entries must be finite")
        ####
    ####

    @property
    def output_dimension(self) -> int:
        return len(self.rows)
    ####

    @property
    def input_dimension(self) -> int:
        return len(self.rows[0]) if self.rows else 0
    ####

    def gradient(self) -> tuple[float, ...]:
        """Return the single output row as a scalar gradient."""

        if self.output_dimension != 1:
            raise ValueError("gradient is only defined for a scalar output")
        return self.rows[0]
    ####


class NewtonSystemStatus(StrEnum):
    """Terminal state of a bounded multidimensional Newton solve."""

    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
####


@dataclass(frozen=True, slots=True)
class NewtonSystemResult:
    """Updated controls and diagnostics from ``TAOS-ALG-GUID-005``."""

    controls: tuple[float, ...]
    residuals: tuple[float, ...]
    iterations: int
    status: NewtonSystemStatus

    @property
    def converged(self) -> bool:
        return self.status is NewtonSystemStatus.CONVERGED
    ####


def newton_system(
    residual_function: Function,
    initial_controls: Sequence[float],
    control_bounds: Sequence[tuple[float, float]],
    increments: Sequence[float] | float,
    residual_tolerance: float,
    *,
    max_iterations: int = 100,
) -> NewtonSystemResult:
    """Solve a bounded residual system using equations 2-281 through 2-283."""

    controls = tuple(float(value) for value in initial_controls)
    bounds = tuple((float(lower), float(upper)) for lower, upper in control_bounds)
    if not controls or len(bounds) != len(controls):
        raise ValueError("controls and bounds must have the same nonzero dimension")
    if any(not math.isfinite(value) for value in controls):
        raise ValueError("controls must be finite")
    if any(lower > upper or not math.isfinite(lower) or not math.isfinite(upper) for lower, upper in bounds):
        raise ValueError("control bounds must be finite and ordered")
    if not math.isfinite(residual_tolerance) or residual_tolerance < 0.0:
        raise ValueError("residual_tolerance must be finite and nonnegative")
    if max_iterations < 0:
        raise ValueError("max_iterations must be nonnegative")
    controls = tuple(min(upper, max(lower, value)) for value, (lower, upper) in zip(controls, bounds, strict=True))
    residuals = _evaluate(residual_function, controls)
    if len(residuals) != len(controls):
        raise ValueError("residual function must return one value per control")
    if _max_abs(residuals) <= residual_tolerance:
        return NewtonSystemResult(controls, residuals, 0, NewtonSystemStatus.CONVERGED)
    ####

    for iteration in range(1, max_iterations + 1):
        jacobian = finite_difference_jacobian(residual_function, controls, increments)
        if jacobian.output_dimension != len(controls) or jacobian.input_dimension != len(controls):
            raise ValueError("residual Jacobian must be square")
        update = guidance_newton_update(controls, residuals, Matrix(jacobian.rows))
        controls = tuple(
            min(upper, max(lower, value))
            for value, (lower, upper) in zip(update, bounds, strict=True)
        )
        residuals = _evaluate(residual_function, controls)
        if _max_abs(residuals) <= residual_tolerance:
            return NewtonSystemResult(controls, residuals, iteration, NewtonSystemStatus.CONVERGED)
        ####
    ####
    return NewtonSystemResult(controls, residuals, max_iterations, NewtonSystemStatus.MAX_ITERATIONS)
####


Function = Callable[[tuple[float, ...]], float | Sequence[float]]


def finite_difference_jacobian(
    function: Function,
    point: Sequence[float],
    increments: Sequence[float] | float,
    *,
    mode: DifferenceMode = DifferenceMode.CENTRAL,
) -> Jacobian:
    """Estimate a scalar gradient or vector-function Jacobian.

    This implements catalog item ``TAOS-ALG-OPT-004`` and the manual's
    forward/central difference displays (equations 4-7 and 4-8). Rows are
    function outputs and columns are perturbed input variables.
    """

    base_point = tuple(float(value) for value in point)
    if not base_point or any(not math.isfinite(value) for value in base_point):
        raise ValueError("point must contain finite values")
    steps = _normalize_increments(increments, len(base_point))
    if not isinstance(mode, DifferenceMode):
        raise ValueError(f"unsupported finite-difference mode: {mode}")
    ####

    base_output = _evaluate(function, base_point)
    columns: list[tuple[float, ...]] = []
    for index, step in enumerate(steps):
        plus_point = list(base_point)
        plus_point[index] += step
        plus_output = _evaluate(function, tuple(plus_point))
        if mode is DifferenceMode.FORWARD:
            column = tuple((plus - base) / step for plus, base in zip(plus_output, base_output, strict=True))
        else:
            minus_point = list(base_point)
            minus_point[index] -= step
            minus_output = _evaluate(function, tuple(minus_point))
            column = tuple((plus - minus) / (2.0 * step) for plus, minus in zip(plus_output, minus_output, strict=True))
        columns.append(column)
    ####
    return Jacobian(tuple(tuple(column[row] for column in columns) for row in range(len(base_output))))
####


def _normalize_increments(increments: Sequence[float] | float, dimension: int) -> tuple[float, ...]:
    if isinstance(increments, (int, float)):
        steps = (float(increments),) * dimension
    else:
        steps = tuple(float(value) for value in increments)
    if len(steps) != dimension or any(not math.isfinite(value) or value == 0.0 for value in steps):
        raise ValueError("increments must contain one finite nonzero value per variable")
    return steps
####


def _evaluate(function: Function, point: tuple[float, ...]) -> tuple[float, ...]:
    raw = function(point)
    if isinstance(raw, (int, float)):
        values: tuple[float, ...] = (float(raw),)
    else:
        values = tuple(float(value) for value in raw)
    if not values or any(not math.isfinite(value) for value in values):
        raise ValueError("function must return finite scalar or vector output")
    return values
####


def _max_abs(values: Sequence[float]) -> float:
    return max(abs(value) for value in values)
####
