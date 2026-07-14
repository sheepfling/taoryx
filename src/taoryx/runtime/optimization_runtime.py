"""Selectable optimization backends for parsed TAOS optimize blocks."""

from __future__ import annotations

import os
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from enum import StrEnum
from importlib.util import find_spec
from typing import Any

from taoryx.numeric import DifferenceMode, finite_difference_jacobian
from taoryx.optimization import OptimizationResult, OptimizationStatus, han_powell_rqp


class OptimizerBackend(StrEnum):
    """Supported optimization implementations."""

    AUTO = "auto"
    BUILTIN_RQP = "builtin-rqp"
    SCIPY_SLSQP = "scipy-slsqp"
    SCIPY_TRUST_CONSTR = "scipy-trust-constr"
    SCIPY_COBYQA = "scipy-cobyqa"
    SCIPY_LBFGSB = "scipy-lbfgsb"
    SCIPY_DIFFERENTIAL_EVOLUTION = "scipy-differential-evolution"
####


class OptimizerUnavailableError(RuntimeError):
    """Raised when an explicitly requested optional backend is unavailable."""
####


Objective = Callable[[tuple[float, ...]], float]
Constraint = Callable[[tuple[float, ...]], float]


def available_optimizers() -> tuple[OptimizerBackend, ...]:
    """Return backends currently importable in this environment."""

    backends = [OptimizerBackend.AUTO, OptimizerBackend.BUILTIN_RQP]
    if find_spec("scipy") is not None:
        from scipy import optimize

        backends.extend((OptimizerBackend.SCIPY_SLSQP, OptimizerBackend.SCIPY_TRUST_CONSTR, OptimizerBackend.SCIPY_LBFGSB))
        if hasattr(optimize, "_minimize_cobyqa"):
            backends.append(OptimizerBackend.SCIPY_COBYQA)
        if hasattr(optimize, "differential_evolution"):
            backends.append(OptimizerBackend.SCIPY_DIFFERENTIAL_EVOLUTION)
    return tuple(backends)
####


def select_optimizer(
    requested: str | OptimizerBackend | None,
    *,
    has_constraints: bool,
) -> OptimizerBackend:
    """Resolve explicit, environment, and automatic backend selection.

    Explicit backend names fail loudly when unavailable. ``auto`` prefers a
    SciPy local optimizer and falls back to the built-in deterministic RQP
    implementation only when SciPy is not installed.
    """

    raw = requested if requested is not None else os.environ.get("TAORYX_OPTIMIZER", OptimizerBackend.AUTO)
    try:
        backend = raw if isinstance(raw, OptimizerBackend) else OptimizerBackend(raw.lower())
    except ValueError as exc:
        choices = ", ".join(item.value for item in OptimizerBackend)
        raise ValueError(f"unknown optimizer backend {raw!r}; choose one of: {choices}") from exc
    if backend is OptimizerBackend.AUTO:
        if find_spec("scipy") is None:
            return OptimizerBackend.BUILTIN_RQP
        return OptimizerBackend.SCIPY_SLSQP if has_constraints else OptimizerBackend.SCIPY_LBFGSB
    if backend.value.startswith("scipy-") and backend not in available_optimizers():
        raise OptimizerUnavailableError(
            f"optimizer {backend.value!r} is unavailable; install a compatible SciPy or select builtin-rqp"
        )
    return backend
####


@dataclass(frozen=True, slots=True)
class OptimizeRuntime:
    """Repeatable optimization runner with a selectable numerical backend."""

    objective: Objective
    bounds: tuple[tuple[float, float], ...]
    equality_constraints: tuple[Constraint, ...] = ()
    inequality_constraints: tuple[Constraint, ...] = ()
    backend: str | OptimizerBackend = OptimizerBackend.AUTO
    tolerance: float = 1e-7
    derivative_step: float = 1e-6
    difference_mode: DifferenceMode = DifferenceMode.FORWARD

    def run(self, initial: Sequence[float], *, max_iterations: int = 100) -> OptimizationResult:
        """Run the selected backend and normalize its result."""

        chosen = select_optimizer(self.backend, has_constraints=bool(self.equality_constraints or self.inequality_constraints))
        if chosen is OptimizerBackend.BUILTIN_RQP:
            return han_powell_rqp(
                self.objective,
                initial,
                self.bounds,
                equality_constraints=self.equality_constraints,
                inequality_constraints=self.inequality_constraints,
                derivative_step=self.derivative_step,
                difference_mode=self.difference_mode,
                tolerance=self.tolerance,
                max_iterations=max_iterations,
            )
        return _run_scipy(
            chosen,
            self.objective,
            initial,
            self.bounds,
            self.equality_constraints,
            self.inequality_constraints,
            tolerance=self.tolerance,
            derivative_step=self.derivative_step,
            difference_mode=self.difference_mode,
            max_iterations=max_iterations,
        )
    ####
####


def resolve_optimize_block(
    objective: Objective,
    bounds: Sequence[tuple[float, float]],
    *,
    equality_constraints: Sequence[Constraint] = (),
    inequality_constraints: Sequence[Constraint] = (),
    backend: str | OptimizerBackend = OptimizerBackend.AUTO,
    tolerance: float = 1e-7,
    derivative_step: float = 1e-6,
    difference_mode: DifferenceMode = DifferenceMode.FORWARD,
) -> OptimizeRuntime:
    """Resolve a typed ``*optimize`` block into a backend-selectable runner."""

    return OptimizeRuntime(
        objective,
        tuple(bounds),
        tuple(equality_constraints),
        tuple(inequality_constraints),
        backend,
        tolerance,
        derivative_step,
        difference_mode,
    )
####


def _run_scipy(
    backend: OptimizerBackend,
    objective: Objective,
    initial: Sequence[float],
    bounds: Sequence[tuple[float, float]],
    equalities: Sequence[Constraint],
    inequalities: Sequence[Constraint],
    *,
    tolerance: float,
    derivative_step: float,
    difference_mode: DifferenceMode,
    max_iterations: int,
) -> OptimizationResult:
    import numpy as np
    from scipy import optimize

    scipy_optimize: Any = optimize

    x0 = np.asarray(tuple(float(value) for value in initial), dtype=float)
    scipy_bounds = tuple((float(lower), float(upper)) for lower, upper in bounds)

    def wrapped(point: Sequence[float]) -> float:
        values = tuple(float(value) for value in point)
        result = float(objective(values))
        if not np.isfinite(result):
            raise ValueError("optimization functions must return finite values")
        return result

    def wrapped_gradient(point: Sequence[float]) -> Any:
        values = tuple(float(value) for value in point)
        return np.asarray(
            finite_difference_jacobian(objective, values, derivative_step, mode=difference_mode).gradient(),
            dtype=float,
        )

    constraints: list[dict[str, Any]] = []
    constraints.extend({"type": "eq", "fun": lambda point, fn=fn: fn(tuple(float(value) for value in point))} for fn in equalities)
    constraints.extend({"type": "ineq", "fun": lambda point, fn=fn: fn(tuple(float(value) for value in point))} for fn in inequalities)

    if backend is OptimizerBackend.SCIPY_DIFFERENTIAL_EVOLUTION:
        result = scipy_optimize.differential_evolution(
            wrapped,
            scipy_bounds,
            maxiter=max_iterations,
            tol=tolerance,
            polish=True,
            seed=0,
            constraints=tuple(
                scipy_optimize.NonlinearConstraint(item["fun"], 0.0, 0.0) if item["type"] == "eq" else scipy_optimize.NonlinearConstraint(item["fun"], 0.0, np.inf)
                for item in constraints
            ),
        )
    else:
        method = {
            OptimizerBackend.SCIPY_SLSQP: "SLSQP",
            OptimizerBackend.SCIPY_TRUST_CONSTR: "trust-constr",
            OptimizerBackend.SCIPY_COBYQA: "COBYQA",
            OptimizerBackend.SCIPY_LBFGSB: "L-BFGS-B",
        }[backend]
        options: dict[str, object] = {"maxiter": max_iterations}
        if method == "SLSQP":
            options["ftol"] = tolerance
            # Trajectory objectives are often evaluated on a finite integration
            # grid; the default SciPy epsilon can produce numerically flat
            # constraint gradients even when the configured runtime step is
            # meaningful.
            options["eps"] = max(derivative_step, 1.0e-12)
        elif method == "trust-constr":
            options["gtol"] = tolerance
        result = scipy_optimize.minimize(
            wrapped,
            x0,
            method=method,
            bounds=scipy_bounds,
            constraints=() if method == "L-BFGS-B" else tuple(constraints),
            jac=None if method == "COBYQA" else wrapped_gradient,
            tol=tolerance,
            options=options,
        )
    parameters = tuple(float(value) for value in result.x)
    equality_values = tuple(_evaluate_constraint(fn, parameters) for fn in equalities)
    inequality_values = tuple(_evaluate_constraint(fn, parameters) for fn in inequalities)
    status = OptimizationStatus.CONVERGED if bool(result.success) else (
        OptimizationStatus.MAX_ITERATIONS if int(getattr(result, "nit", max_iterations) or 0) >= max_iterations else OptimizationStatus.STALLED
    )
    return OptimizationResult(
        parameters,
        _evaluate_objective(objective, parameters),
        equality_values,
        inequality_values,
        int(getattr(result, "nit", 0) or 0),
        status,
    )
####


def _evaluate_objective(function: Objective, point: tuple[float, ...]) -> float:
    value = float(function(point))
    if not value == value or value in (float("inf"), float("-inf")):
        raise ValueError("optimization functions must return finite values")
    return value
####


def _evaluate_constraint(function: Constraint, point: tuple[float, ...]) -> float:
    return _evaluate_objective(function, point)
####
