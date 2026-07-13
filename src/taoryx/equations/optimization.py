"""Optimization equation helpers from the TAOS manual."""

from __future__ import annotations

from dataclasses import dataclass

from .guidance import Matrix


@dataclass(frozen=True, slots=True)
class NonlinearProgram:
    """Minimal representation of a constrained nonlinear program."""

    objective: str
    equality_constraints: tuple[str, ...]
    inequality_constraints: tuple[str, ...]
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
) -> NonlinearProgram:
    """Return a structured nonlinear-program representation."""

    return NonlinearProgram(objective, equality_constraints, inequality_constraints)
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
