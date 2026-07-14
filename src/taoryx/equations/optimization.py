"""Optimization equation helpers from the TAOS manual."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .guidance import Matrix


@dataclass(frozen=True, slots=True)
class NonlinearProgram:
    """Minimal representation of a constrained nonlinear program."""

    objective: str
    equality_constraints: tuple[str, ...]
    inequality_constraints: tuple[str, ...]
    parameters: tuple[str, ...] = ()
    bounds: tuple[tuple[float, float], ...] = ()
    references: tuple[float, ...] = ()
####


@dataclass(frozen=True, slots=True)
class EqualityConstrainedProblem:
    """Minimal representation of a fixed active-set equality problem."""

    objective: str
    constraints: tuple[str, ...]
####


def _dot(left: tuple[float, ...], right: tuple[float, ...]) -> float:
    return sum(component * value for component, value in zip(left, right, strict=True))
####


def _matrix_vector_product(matrix: Matrix, vector: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(sum(component * value for component, value in zip(row, vector, strict=True)) for row in matrix.rows)
####


def _quadratic_form(vector: tuple[float, ...], matrix: Matrix) -> float:
    return _dot(vector, _matrix_vector_product(matrix, vector))
####


def general_nonlinear_program(
    objective: str,
    equality_constraints: tuple[str, ...],
    inequality_constraints: tuple[str, ...],
    parameters: tuple[str, ...] = (),
    bounds: tuple[tuple[float, float], ...] = (),
    references: tuple[float, ...] = (),
) -> NonlinearProgram:
    """Return a structured nonlinear-program representation."""

    if len(parameters) != len(bounds):
        raise ValueError("optimization parameters and bounds must have matching lengths")
    if any(lower > upper or not math.isfinite(lower) or not math.isfinite(upper) for lower, upper in bounds):
        raise ValueError("optimization bounds must be finite and ordered")
    if any(not math.isfinite(reference) or reference == 0.0 for reference in references):
        raise ValueError("optimization references must be finite and nonzero")
    return NonlinearProgram(objective, equality_constraints, inequality_constraints, parameters, bounds, references)
####


def active_set_equality_problem(objective: str, constraints: tuple[str, ...]) -> EqualityConstrainedProblem:
    """Return a structured active-set equality problem."""

    return EqualityConstrainedProblem(objective, constraints)
####


def optimization_quadratic_approximation(
    objective_gradient: tuple[float, ...],
    objective_hessian: Matrix,
    delta_x: tuple[float, ...],
) -> float:
    """Return the quadratic approximation of the objective change."""

    return _dot(objective_gradient, delta_x) + 0.5 * _quadratic_form(delta_x, objective_hessian)
####


def optimization_constraint_linearization(
    constraint_value: tuple[float, ...],
    constraint_jacobian: Matrix,
    delta_x: tuple[float, ...],
) -> tuple[float, ...]:
    """Return the linearized constraint values."""

    increment = _matrix_vector_product(constraint_jacobian, delta_x)
    return tuple(value + delta for value, delta in zip(constraint_value, increment, strict=True))
####


def optimization_quadratic_subproblem(
    objective_gradient: tuple[float, ...],
    variable_metric_matrix: Matrix,
    delta_x: tuple[float, ...],
    constraint_value: tuple[float, ...],
    constraint_jacobian: Matrix,
) -> tuple[float, tuple[float, ...]]:
    """Return the quadratic objective and linearized constraints for the subproblem."""

    return (
        optimization_quadratic_approximation(objective_gradient, variable_metric_matrix, delta_x),
        optimization_constraint_linearization(constraint_value, constraint_jacobian, delta_x),
    )
####


def maximum_altitude_integral(altitude: float, threshold: float = 100000.0) -> float:
    """Return the path integral contribution for the maximum-altitude constraint."""

    if altitude > threshold:
        return (altitude - threshold) * (altitude - threshold)
    ####
    return 0.0
####


def maximum_altitude_constraint(value: float) -> bool:
    """Return whether the strict maximum-altitude constraint is satisfied."""

    return value < 0.0
####


def optimization_forward_difference(function_plus: float, function_zero: float, delta_x: float) -> float:
    """Return the forward-difference numerical derivative."""

    return (function_plus - function_zero) / delta_x
####


def optimization_central_difference(function_plus: float, function_minus: float, delta_x: float) -> float:
    """Return the central-difference numerical derivative."""

    return (function_plus - function_minus) / (2.0 * delta_x)
####


def qmin_satisfied() -> float:
    """Return the minimum-dynamic-pressure integral derivative when satisfied."""

    return 0.0
####


def qmin_violated(minimum_pressure: float, dynamic_pressure: float) -> float:
    """Return the minimum-dynamic-pressure integral derivative when violated."""

    return (minimum_pressure - dynamic_pressure) * (minimum_pressure - dynamic_pressure)
####


def stagnation_heating(nose_radius: float, air_density: float, sea_level_density: float, vehicle_speed: float, satellite_speed: float = 26000.0) -> float:
    """Return the stagnation-point heat-transfer approximation."""

    return (17600.0 / math.sqrt(nose_radius)) * math.sqrt(air_density / sea_level_density) * (vehicle_speed / satellite_speed) ** 3.15
####


def optimization_qmin_satisfied(minimum_pressure: float, dynamic_pressure: float, velocity: float) -> float:
    """Return the *optimize qmin derivative when satisfied."""

    if dynamic_pressure > minimum_pressure or velocity < 1000.0:
        return 0.0
    ####
    return (minimum_pressure - dynamic_pressure) * (minimum_pressure - dynamic_pressure)
####


def optimization_qmin_violated(minimum_pressure: float, dynamic_pressure: float) -> float:
    """Return the *optimize qmin derivative when violated."""

    return (minimum_pressure - dynamic_pressure) * (minimum_pressure - dynamic_pressure)
####
