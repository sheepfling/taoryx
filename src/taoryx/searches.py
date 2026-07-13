"""Bounded one-dimensional search algorithms from the TAOS catalog."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum


class RootSearchStatus(StrEnum):
    """Terminal state of a bounded root search."""

    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
    ZERO_DERIVATIVE = "zero_derivative"
    BOUNDARY_STALL = "boundary_stall"
    INVALID_BRACKET = "invalid_bracket"
    ZERO_SECANT_DENOMINATOR = "zero_secant_denominator"
    BRACKET_STALL = "bracket_stall"
    DEGENERATE_PARABOLA = "degenerate_parabola"
    NO_VALID_ROOT = "no_valid_root"
####


class MinimizationStatus(StrEnum):
    """Terminal state of a bounded one-dimensional minimization."""

    CONVERGED = "converged"
    MAX_ITERATIONS = "max_iterations"
####


@dataclass(frozen=True, slots=True)
class RootSearchResult:
    """Root estimate and diagnostics returned by an iterative search."""

    root: float
    residual: float
    iterations: int
    status: RootSearchStatus

    @property
    def converged(self) -> bool:
        return self.status is RootSearchStatus.CONVERGED
    ####


@dataclass(frozen=True, slots=True)
class BracketedRootSearchResult:
    """Secant root estimate together with the retained sign-change interval."""

    root: float
    residual: float
    iterations: int
    status: RootSearchStatus
    bracket: tuple[float, float]

    @property
    def converged(self) -> bool:
        return self.status is RootSearchStatus.CONVERGED
    ####


@dataclass(frozen=True, slots=True)
class MinimizationResult:
    """Minimum estimate, objective value, and final enclosing interval."""

    minimum: float
    value: float
    iterations: int
    status: MinimizationStatus
    interval: tuple[float, float]

    @property
    def converged(self) -> bool:
        return self.status is MinimizationStatus.CONVERGED
    ####


def newton_root(
    function: Callable[[float], float],
    initial_estimate: float,
    bounds: tuple[float, float],
    derivative_step: float,
    residual_tolerance: float,
    *,
    max_iterations: int = 100,
) -> RootSearchResult:
    """Solve ``f(x)=0`` with bounded forward-difference Newton iteration.

    This implements ``TAOS-ALG-SEARCH-001`` from equations 2-298 and 2-299.
    Candidate updates are clipped to the supplied interval. A terminal result
    is returned for numerical failure so callers can choose whether a failed
    search should abort, retry, or become a runtime diagnostic.
    """

    lower, upper = bounds
    _validate_inputs(initial_estimate, lower, upper, derivative_step, residual_tolerance, max_iterations)
    x = initial_estimate
    residual = _evaluate(function, x)
    if abs(residual) <= residual_tolerance:
        return RootSearchResult(x, residual, 0, RootSearchStatus.CONVERGED)
    ####

    for iteration in range(1, max_iterations + 1):
        forward_x = min(x + derivative_step, upper)
        if forward_x == x:
            return RootSearchResult(x, residual, iteration - 1, RootSearchStatus.BOUNDARY_STALL)
        ####
        forward_residual = _evaluate(function, forward_x)
        derivative = (forward_residual - residual) / (forward_x - x)
        if math.isclose(derivative, 0.0, abs_tol=1e-15, rel_tol=0.0):
            return RootSearchResult(x, residual, iteration - 1, RootSearchStatus.ZERO_DERIVATIVE)
        ####
        candidate = min(upper, max(lower, x - residual / derivative))
        if candidate == x:
            return RootSearchResult(x, residual, iteration, RootSearchStatus.BOUNDARY_STALL)
        ####
        x = candidate
        residual = _evaluate(function, x)
        if abs(residual) <= residual_tolerance:
            return RootSearchResult(x, residual, iteration, RootSearchStatus.CONVERGED)
        ####
    ####
    return RootSearchResult(x, residual, max_iterations, RootSearchStatus.MAX_ITERATIONS)
####


def secant_bracketed_root(
    function: Callable[[float], float],
    bounds: tuple[float, float],
    residual_tolerance: float,
    *,
    max_iterations: int = 100,
) -> BracketedRootSearchResult:
    """Solve a sign-bracketed root with the TAOS secant update.

    This implements ``TAOS-ALG-SEARCH-002`` and equation 2-300. Every update
    retains the endpoint pair whose function values have opposite signs.
    """

    lower, upper = bounds
    _validate_bracket_inputs(lower, upper, residual_tolerance, max_iterations)
    lower_value = _evaluate(function, lower)
    upper_value = _evaluate(function, upper)
    if abs(lower_value) <= residual_tolerance:
        return BracketedRootSearchResult(lower, lower_value, 0, RootSearchStatus.CONVERGED, (lower, lower))
    if abs(upper_value) <= residual_tolerance:
        return BracketedRootSearchResult(upper, upper_value, 0, RootSearchStatus.CONVERGED, (upper, upper))
    if lower_value * upper_value > 0.0:
        return BracketedRootSearchResult(lower, lower_value, 0, RootSearchStatus.INVALID_BRACKET, (lower, upper))
    ####

    for iteration in range(1, max_iterations + 1):
        denominator = upper_value - lower_value
        if math.isclose(denominator, 0.0, abs_tol=1e-15, rel_tol=0.0):
            root, residual = _best_endpoint(lower, lower_value, upper, upper_value)
            return BracketedRootSearchResult(root, residual, iteration - 1, RootSearchStatus.ZERO_SECANT_DENOMINATOR, (lower, upper))
        ####
        estimate = lower - lower_value * (upper - lower) / denominator
        if not lower < estimate < upper:
            root, residual = _best_endpoint(lower, lower_value, upper, upper_value)
            return BracketedRootSearchResult(root, residual, iteration - 1, RootSearchStatus.BRACKET_STALL, (lower, upper))
        ####
        estimate_value = _evaluate(function, estimate)
        if abs(estimate_value) <= residual_tolerance:
            return BracketedRootSearchResult(estimate, estimate_value, iteration, RootSearchStatus.CONVERGED, (lower, upper))
        ####
        if lower_value * estimate_value < 0.0:
            upper, upper_value = estimate, estimate_value
        else:
            lower, lower_value = estimate, estimate_value
        ####
    ####
    root, residual = _best_endpoint(lower, lower_value, upper, upper_value)
    return BracketedRootSearchResult(root, residual, max_iterations, RootSearchStatus.MAX_ITERATIONS, (lower, upper))
####


def golden_section_minimize(
    function: Callable[[float], float],
    bounds: tuple[float, float],
    interval_tolerance: float,
    *,
    max_iterations: int = 1000,
) -> MinimizationResult:
    """Minimize a bounded scalar function without derivatives.

    This implements ``TAOS-ALG-SEARCH-004`` and equations 2-306 and 2-307.
    The retained interval always contains the current best interior point and
    contracts by the golden ratio on every iteration.
    """

    lower, upper = bounds
    _validate_minimization_inputs(lower, upper, interval_tolerance, max_iterations)
    ratio = (math.sqrt(5.0) - 1.0) / 2.0
    first = upper - ratio * (upper - lower)
    second = lower + ratio * (upper - lower)
    first_value = _evaluate(function, first)
    second_value = _evaluate(function, second)
    iteration = 0
    while upper - lower > interval_tolerance and iteration < max_iterations:
        if first_value > second_value:
            lower = first
            first = second
            first_value = second_value
            second = lower + ratio * (upper - lower)
            second_value = _evaluate(function, second)
        else:
            upper = second
            second = first
            second_value = first_value
            first = upper - ratio * (upper - lower)
            first_value = _evaluate(function, first)
        iteration += 1
        ####
    ####
    if first_value <= second_value:
        minimum, value = first, first_value
    else:
        minimum, value = second, second_value
    status = MinimizationStatus.CONVERGED if upper - lower <= interval_tolerance else MinimizationStatus.MAX_ITERATIONS
    return MinimizationResult(minimum, value, iteration, status, (lower, upper))
####


def parabolic_root(
    function: Callable[[float], float],
    initial_estimate: float,
    spacing: float,
    bounds: tuple[float, float],
    residual_tolerance: float,
    *,
    max_iterations: int = 100,
) -> RootSearchResult:
    """Solve a bounded root using the TAOS three-point parabola.

    This implements TAOS-ALG-SEARCH-003 and equations 2-301 through 2-305.
    """

    lower, upper = bounds
    _validate_inputs(initial_estimate, lower, upper, spacing, residual_tolerance, max_iterations)
    points = _startup_points(initial_estimate, spacing, lower, upper)
    values = [_evaluate(function, point) for point in points]
    for iteration in range(max_iterations + 1):
        best_index = min(range(3), key=lambda index: abs(values[index]))
        if abs(values[best_index]) <= residual_tolerance:
            return RootSearchResult(points[best_index], values[best_index], iteration, RootSearchStatus.CONVERGED)
        coefficients = _quadratic_coefficients(points, values)
        if coefficients is None:
            return RootSearchResult(points[best_index], values[best_index], iteration, RootSearchStatus.DEGENERATE_PARABOLA)
        candidates = [root for root in _quadratic_roots(*coefficients) if lower <= root <= upper]
        if not candidates:
            return RootSearchResult(points[best_index], values[best_index], iteration, RootSearchStatus.NO_VALID_ROOT)
        candidate = min(candidates, key=lambda root: abs(root - points[best_index]))
        if any(math.isclose(candidate, point, abs_tol=1e-15, rel_tol=0.0) for point in points):
            return RootSearchResult(points[best_index], values[best_index], iteration, RootSearchStatus.BOUNDARY_STALL)
        candidate_value = _evaluate(function, candidate)
        if abs(candidate_value) <= residual_tolerance:
            return RootSearchResult(candidate, candidate_value, iteration + 1, RootSearchStatus.CONVERGED)
        discard = max(range(3), key=lambda index: abs(points[index] - candidate))
        points[discard] = candidate
        values[discard] = candidate_value
    ####
    best_index = min(range(3), key=lambda index: abs(values[index]))
    return RootSearchResult(points[best_index], values[best_index], max_iterations, RootSearchStatus.MAX_ITERATIONS)
####


def parabolic_minimize(
    function: Callable[[float], float],
    initial_estimate: float,
    spacing: float,
    bounds: tuple[float, float],
    function_agreement_tolerance: float,
    *,
    max_iterations: int = 100,
) -> MinimizationResult:
    """Minimize a bounded function using the TAOS three-point parabola."""

    lower, upper = bounds
    if not math.isfinite(function_agreement_tolerance) or function_agreement_tolerance <= 0.0:
        raise ValueError("function agreement tolerance must be positive and finite")
    _validate_inputs(initial_estimate, lower, upper, spacing, function_agreement_tolerance, max_iterations)
    points = _startup_points(initial_estimate, spacing, lower, upper)
    values = [_evaluate(function, point) for point in points]
    for iteration in range(1, max_iterations + 1):
        coefficients = _quadratic_coefficients(points, values)
        if coefficients is None or coefficients[0] <= 0.0:
            return _best_minimum(points, values, iteration - 1, RootSearchStatus.DEGENERATE_PARABOLA)
        candidate = -coefficients[1] / (2.0 * coefficients[0])
        if not lower <= candidate <= upper:
            return _best_minimum(points, values, iteration - 1, RootSearchStatus.NO_VALID_ROOT)
        candidate_value = _evaluate(function, candidate)
        predicted_value = coefficients[0] * candidate * candidate + coefficients[1] * candidate + coefficients[2]
        if abs(candidate_value - predicted_value) <= function_agreement_tolerance:
            return MinimizationResult(candidate, candidate_value, iteration, MinimizationStatus.CONVERGED, (min(points), max(points)))
        discard = max(range(3), key=lambda index: values[index])
        points[discard] = candidate
        values[discard] = candidate_value
    ####
    return _best_minimum(points, values, max_iterations, RootSearchStatus.MAX_ITERATIONS)
####


def _validate_inputs(
    initial_estimate: float,
    lower: float,
    upper: float,
    derivative_step: float,
    residual_tolerance: float,
    max_iterations: int,
) -> None:
    values = (initial_estimate, lower, upper, derivative_step, residual_tolerance)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("root-search inputs must be finite")
    if lower >= upper:
        raise ValueError("root-search lower bound must be less than upper bound")
    if not lower <= initial_estimate <= upper:
        raise ValueError("initial estimate must lie within root-search bounds")
    if derivative_step <= 0.0:
        raise ValueError("derivative step must be positive")
    if residual_tolerance < 0.0:
        raise ValueError("residual tolerance must be nonnegative")
    if max_iterations < 1:
        raise ValueError("max_iterations must be positive")
####


def _startup_points(initial_estimate: float, spacing: float, lower: float, upper: float) -> list[float]:
    points = [max(lower, initial_estimate - spacing), initial_estimate, min(upper, initial_estimate + spacing)]
    if len(set(points)) != 3:
        raise ValueError("parabolic search requires three distinct startup points")
    return points
####


def _quadratic_coefficients(points: list[float], values: list[float]) -> tuple[float, float, float] | None:
    x0, x1, x2 = points
    y0, y1, y2 = values
    denominator = (x0 - x1) * (x0 - x2) * (x1 - x2)
    if math.isclose(denominator, 0.0, abs_tol=1e-15, rel_tol=0.0):
        return None
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denominator
    b = (x2 * x2 * (y0 - y1) + x1 * x1 * (y2 - y0) + x0 * x0 * (y1 - y2)) / denominator
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denominator
    return a, b, c
####


def _quadratic_roots(a: float, b: float, c: float) -> tuple[float, ...]:
    if math.isclose(a, 0.0, abs_tol=1e-15, rel_tol=0.0):
        if math.isclose(b, 0.0, abs_tol=1e-15, rel_tol=0.0):
            return ()
        return (-c / b,)
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return ()
    root = math.sqrt(discriminant)
    return ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))
####


def _best_minimum(points: list[float], values: list[float], iterations: int, status: RootSearchStatus) -> MinimizationResult:
    index = min(range(3), key=lambda item: values[item])
    return MinimizationResult(points[index], values[index], iterations, MinimizationStatus.MAX_ITERATIONS, (min(points), max(points)))
####


def _validate_bracket_inputs(lower: float, upper: float, residual_tolerance: float, max_iterations: int) -> None:
    values = (lower, upper, residual_tolerance)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("bracket-search inputs must be finite")
    if lower >= upper:
        raise ValueError("bracket lower bound must be less than upper bound")
    if residual_tolerance < 0.0:
        raise ValueError("residual tolerance must be nonnegative")
    if max_iterations < 1:
        raise ValueError("max_iterations must be positive")
####


def _validate_minimization_inputs(lower: float, upper: float, interval_tolerance: float, max_iterations: int) -> None:
    values = (lower, upper, interval_tolerance)
    if any(not math.isfinite(value) for value in values):
        raise ValueError("minimization inputs must be finite")
    if lower >= upper:
        raise ValueError("minimization lower bound must be less than upper bound")
    if interval_tolerance <= 0.0:
        raise ValueError("interval tolerance must be positive")
    if max_iterations < 1:
        raise ValueError("max_iterations must be positive")
####


def _evaluate(function: Callable[[float], float], x: float) -> float:
    value = float(function(x))
    if not math.isfinite(value):
        raise ValueError("root-search function must return a finite value")
    return value
####


def _best_endpoint(lower: float, lower_value: float, upper: float, upper_value: float) -> tuple[float, float]:
    return (lower, lower_value) if abs(lower_value) <= abs(upper_value) else (upper, upper_value)
####
