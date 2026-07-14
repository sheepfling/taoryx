"""Typed optimization contracts for the TAOS trajectory workflow."""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import StrEnum

from .equations import NonlinearProgram, general_nonlinear_program
from .numeric import DifferenceMode, finite_difference_jacobian


class OptimizationStatus(StrEnum):
    """Terminal state of the bounded RQP-style optimization loop."""

    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
    STALLED = "stalled"
####


@dataclass(frozen=True, slots=True)
class OptimizationResult:
    """Solution vector and constraint diagnostics from one optimization loop."""

    parameters: tuple[float, ...]
    objective: float
    equality_residuals: tuple[float, ...]
    inequality_values: tuple[float, ...]
    iterations: int
    status: OptimizationStatus

    @property
    def converged(self) -> bool:
        return self.status is OptimizationStatus.CONVERGED
    ####


def build_optimization_problem(
    objective: str,
    equality_constraints: Sequence[str] = (),
    inequality_constraints: Sequence[str] = (),
    *,
    parameters: Sequence[str] = (),
    bounds: Sequence[tuple[float, float]] = (),
    references: Sequence[float] = (),
) -> NonlinearProgram:
    """Evaluate TAOS-ALG-OPT-001's typed nonlinear-program boundary."""

    return general_nonlinear_program(
        objective,
        tuple(equality_constraints),
        tuple(inequality_constraints),
        tuple(parameters),
        tuple((float(lower), float(upper)) for lower, upper in bounds),
        tuple(float(reference) for reference in references),
    )
####


def han_powell_rqp(
    objective: Callable[[tuple[float, ...]], float],
    initial_parameters: Sequence[float],
    bounds: Sequence[tuple[float, float]],
    *,
    equality_constraints: Sequence[Callable[[tuple[float, ...]], float]] = (),
    inequality_constraints: Sequence[Callable[[tuple[float, ...]], float]] = (),
    derivative_step: float = 1e-6,
    difference_mode: DifferenceMode = DifferenceMode.FORWARD,
    tolerance: float = 1e-7,
    max_iterations: int = 100,
) -> OptimizationResult:
    """Run a bounded RQP-style penalty iteration for TAOS-ALG-OPT-002.

    This is a deterministic typed optimizer contract, not a claim to reproduce
    the historical proprietary ``vf02ad`` implementation. It uses numerical
    gradients, a quadratic penalty for constraint violations, projected updates,
    and backtracking to provide a transparent fallback runtime.
    """

    parameters = tuple(float(value) for value in initial_parameters)
    limits = tuple((float(lower), float(upper)) for lower, upper in bounds)
    if not parameters or len(limits) != len(parameters):
        raise ValueError("parameters and bounds must have the same nonzero dimension")
    if any(lower > upper or not math.isfinite(lower) or not math.isfinite(upper) for lower, upper in limits):
        raise ValueError("optimization bounds must be finite and ordered")
    if derivative_step <= 0.0 or not math.isfinite(derivative_step):
        raise ValueError("derivative_step must be positive and finite")
    if tolerance <= 0.0 or not math.isfinite(tolerance):
        raise ValueError("tolerance must be positive and finite")
    if max_iterations < 0:
        raise ValueError("max_iterations must be nonnegative")
    parameters = _project(parameters, limits)
    penalty = 10.0
    objective = _cached_point_function(objective)
    equality_constraints = tuple(_cached_point_function(function) for function in equality_constraints)
    inequality_constraints = tuple(_cached_point_function(function) for function in inequality_constraints)
    for iteration in range(max_iterations + 1):
        objective_value = _evaluate(objective, parameters)
        equalities = tuple(_evaluate(function, parameters) for function in equality_constraints)
        inequalities = tuple(_evaluate(function, parameters) for function in inequality_constraints)
        violation = _constraint_violation(equalities, inequalities)
        objective_gradient = finite_difference_jacobian(objective, parameters, derivative_step, mode=difference_mode).gradient()
        if violation <= tolerance and max(abs(value) for value in objective_gradient) <= tolerance:
            return OptimizationResult(parameters, objective_value, equalities, inequalities, iteration, OptimizationStatus.CONVERGED)
        if iteration == max_iterations:
            break
        ####
        def merit(point: tuple[float, ...]) -> float:
            equal = tuple(_evaluate(function, point) for function in equality_constraints)
            inequal = tuple(_evaluate(function, point) for function in inequality_constraints)
            return _evaluate(objective, point) + penalty * _constraint_penalty(equal, inequal)

        gradient = finite_difference_jacobian(merit, parameters, derivative_step, mode=difference_mode).gradient()
        current_merit = merit(parameters)
        step = 1.0
        while step >= 1e-8:
            candidate = _project(tuple(value - step * component for value, component in zip(parameters, gradient, strict=True)), limits)
            if merit(candidate) < current_merit:
                parameters = candidate
                break
            step *= 0.5
        else:
            return OptimizationResult(parameters, objective_value, equalities, inequalities, iteration, OptimizationStatus.STALLED)
        ####
        penalty *= 1.05
    ####
    equalities = tuple(_evaluate(function, parameters) for function in equality_constraints)
    inequalities = tuple(_evaluate(function, parameters) for function in inequality_constraints)
    return OptimizationResult(
        parameters,
        _evaluate(objective, parameters),
        equalities,
        inequalities,
        max_iterations,
        OptimizationStatus.MAX_ITERATIONS,
    )
####


def path_violation_integral(
    values: Sequence[float],
    *,
    lower: float | None = None,
    upper: float | None = None,
    step_size: float = 1.0,
) -> float:
    """Evaluate TAOS-ALG-OPT-003's squared path-limit violation integral."""

    if step_size <= 0.0 or not math.isfinite(step_size):
        raise ValueError("step_size must be positive and finite")
    if lower is None and upper is None:
        raise ValueError("at least one path limit is required")
    total = 0.0
    for value in values:
        normalized = float(value)
        if not math.isfinite(normalized):
            raise ValueError("path values must be finite")
        violation = max(0.0, (lower - normalized) if lower is not None else 0.0, (normalized - upper) if upper is not None else 0.0)
        total += violation * violation * step_size
    return total
####


def redistribute_control_history(
    times: Sequence[float],
    controls: Sequence[Sequence[float]],
    new_times: Sequence[float],
) -> tuple[tuple[float, ...], ...]:
    """Evaluate TAOS-ALG-OPT-005 by linearly redistributing control history."""

    source_times = tuple(float(value) for value in times)
    target_times = tuple(float(value) for value in new_times)
    rows = tuple(tuple(float(value) for value in row) for row in controls)
    if len(source_times) != len(rows) or len(source_times) < 2 or any(not math.isfinite(value) for value in source_times + target_times):
        raise ValueError("control history requires finite times and at least two samples")
    if any(left >= right for left, right in zip(source_times, source_times[1:], strict=False)):
        raise ValueError("control history times must be strictly increasing")
    width = len(rows[0])
    if width == 0 or any(len(row) != width for row in rows):
        raise ValueError("control history rows must have equal nonzero width")
    return tuple(tuple(_linear_interpolate(source_times, tuple(row[index] for row in rows), time) for index in range(width)) for time in target_times)
####


def _evaluate(function: Callable[[tuple[float, ...]], float], point: tuple[float, ...]) -> float:
    value = float(function(point))
    if not math.isfinite(value):
        raise ValueError("optimization functions must return finite values")
    return value
####


def _constraint_penalty(equalities: Sequence[float], inequalities: Sequence[float]) -> float:
    return sum(value * value for value in equalities) + sum(max(0.0, -value) ** 2 for value in inequalities)
####


def _constraint_violation(equalities: Sequence[float], inequalities: Sequence[float]) -> float:
    equality_violation = max((abs(value) for value in equalities), default=0.0)
    inequality_violation = max((max(0.0, -value) for value in inequalities), default=0.0)
    return max(equality_violation, inequality_violation)
####


def _project(values: Sequence[float], bounds: Sequence[tuple[float, float]]) -> tuple[float, ...]:
    return tuple(min(upper, max(lower, value)) for value, (lower, upper) in zip(values, bounds, strict=True))
####


def _linear_interpolate(times: tuple[float, ...], values: tuple[float, ...], target: float) -> float:
    if target <= times[0]:
        return values[0]
    if target >= times[-1]:
        return values[-1]
    index = next(index for index in range(len(times) - 1) if times[index] <= target <= times[index + 1])
    fraction = (target - times[index]) / (times[index + 1] - times[index])
    return values[index] + fraction * (values[index + 1] - values[index])
####


def _cached_point_function(function: Callable[[tuple[float, ...]], float]) -> Callable[[tuple[float, ...]], float]:
    """Memoize repeated scalar evaluations at the same candidate point."""

    cache: dict[tuple[float, ...], float] = {}

    def cached(point: tuple[float, ...]) -> float:
        if point not in cache:
            cache[point] = _evaluate(function, point)
        return cache[point]
    ####

    return cached
####
