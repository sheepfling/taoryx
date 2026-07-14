"""Typed numerical integration algorithms from the TAOS catalog."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from enum import StrEnum
from importlib.util import find_spec

from .simulation.contracts import DerivativeModel, SimulationState


class IntegratorName(StrEnum):
    """Stable runtime names for reference and optional numerical integrators."""

    EULER = "euler"
    RK4 = "rk4"
    RKF45 = "rkf45"
    SCIPY_RK45 = "scipy-rk45"
    SCIPY_DOP853 = "scipy-dop853"
    SCIPY_RADAU = "scipy-radau"
    SCIPY_BDF = "scipy-bdf"
    SCIPY_LSODA = "scipy-lsoda"
####


_SCIPY_METHODS = {
    IntegratorName.SCIPY_RK45: "RK45",
    IntegratorName.SCIPY_DOP853: "DOP853",
    IntegratorName.SCIPY_RADAU: "Radau",
    IntegratorName.SCIPY_BDF: "BDF",
    IntegratorName.SCIPY_LSODA: "LSODA",
}


def normalize_integrator(name: str | IntegratorName) -> IntegratorName:
    """Normalize a public integrator name and reject unsupported values."""

    try:
        return name if isinstance(name, IntegratorName) else IntegratorName(name.casefold())
    except ValueError as error:
        choices = ", ".join(item.value for item in IntegratorName)
        raise ValueError(f"unknown integrator {name!r}; choose one of: {choices}") from error
####


def available_integrators() -> tuple[IntegratorName, ...]:
    """List public integrators and SciPy methods available in this environment."""

    result = [IntegratorName.EULER, IntegratorName.RK4, IntegratorName.RKF45]
    if find_spec("scipy") is not None:
        result.extend(_SCIPY_METHODS)
    return tuple(result)
####


def euler_step(model: DerivativeModel, state: SimulationState, step_size: float) -> SimulationState:
    """Advance one state with explicit Euler.

    This is intentionally exposed for fast smoke tests and transparent
    derivative-pipeline checks, not as the recommended production method.
    """

    if not math.isfinite(step_size) or step_size <= 0.0:
        raise ValueError("step_size must be positive and finite")
    rates = _evaluate(model, state)
    values = tuple(value + step_size * rate for value, rate in zip(state.values, rates, strict=True))
    return SimulationState(state.time + step_size, values, state.frame)
####


def rk4_step(model: DerivativeModel, state: SimulationState, step_size: float) -> SimulationState:
    """Advance one state with the classical fixed-step RK4 method.

    This implements TAOS-ALG-DYN-003 and equations 2-114 through 2-118.
    The callback receives time-tagged intermediate states, so stage times and
    the state frame remain explicit at the integration boundary.
    """

    if not math.isfinite(step_size) or step_size <= 0.0:
        raise ValueError("step_size must be positive and finite")
    k1 = _evaluate(model, state)
    midpoint_one = _advance(state, 0.5 * step_size, k1)
    k2 = _evaluate(model, midpoint_one)
    midpoint_two = _advance(state, 0.5 * step_size, k2)
    k3 = _evaluate(model, midpoint_two)
    endpoint = _advance(state, step_size, k3)
    k4 = _evaluate(model, endpoint)
    values = tuple(
        value + step_size * (first + 2.0 * second + 2.0 * third + fourth) / 6.0
        for value, first, second, third, fourth in zip(state.values, k1, k2, k3, k4, strict=True)
    )
    return SimulationState(state.time + step_size, values, state.frame)
####


@dataclass(frozen=True, slots=True)
class RK4Integrator:
    """Integrator-protocol adapter for the catalog RK4 step."""

    def step(self, model: DerivativeModel, state: SimulationState, step_size: float) -> SimulationState:
        return rk4_step(model, state, step_size)
####


def scipy_ivp_step(
    model: DerivativeModel,
    state: SimulationState,
    step_size: float,
    absolute_tolerance: float,
    relative_tolerance: float,
    *,
    method: str,
) -> SimulationState:
    """Advance one engine step with an optional SciPy ``solve_ivp`` method.

    The runtime still owns segment boundaries and event refinement. SciPy is
    therefore constrained to the current engine step rather than replacing the
    scheduling contract with its own event loop.
    """

    if not math.isfinite(step_size) or step_size <= 0.0:
        raise ValueError("step_size must be positive and finite")
    if not math.isfinite(absolute_tolerance) or absolute_tolerance <= 0.0:
        raise ValueError("absolute_tolerance must be positive and finite")
    if not math.isfinite(relative_tolerance) or relative_tolerance < 0.0:
        raise ValueError("relative_tolerance must be finite and nonnegative")
    try:
        from scipy.integrate import solve_ivp
    except ImportError as error:
        raise RuntimeError("SciPy integrators require the optional 'scipy' dependency") from error

    def wrapped(time: float, values: Sequence[float]) -> tuple[float, ...]:
        current = SimulationState(time, tuple(float(value) for value in values), state.frame)
        return tuple(float(value) for value in model(current))
    ####

    result = solve_ivp(
        wrapped,
        (state.time, state.time + step_size),
        state.values,
        method=method,
        rtol=relative_tolerance,
        atol=absolute_tolerance,
        max_step=step_size,
        t_eval=(state.time + step_size,),
    )
    if not result.success or result.y.shape[1] == 0:
        raise RuntimeError(f"SciPy {method} integration failed: {result.message}")
    return SimulationState(state.time + step_size, tuple(float(value) for value in result.y[:, -1]), state.frame)
####


@dataclass(frozen=True, slots=True)
class RKF45Result:
    """Accepted Fehlberg step and its adaptive-step diagnostics."""

    state: SimulationState
    accepted_step: float
    next_step: float
    error_norm: float
    rejected_steps: int
####


def rkf45_step(
    model: DerivativeModel,
    state: SimulationState,
    step_size: float,
    absolute_tolerance: float,
    relative_tolerance: float,
    *,
    max_rejections: int = 20,
) -> RKF45Result:
    """Evaluate TAOS-ALG-IIP-002 with the standard Fehlberg 4(5) tableau."""

    if not math.isfinite(step_size) or step_size <= 0.0:
        raise ValueError("step_size must be positive and finite")
    if not math.isfinite(absolute_tolerance) or absolute_tolerance <= 0.0:
        raise ValueError("absolute_tolerance must be positive and finite")
    if not math.isfinite(relative_tolerance) or relative_tolerance < 0.0:
        raise ValueError("relative_tolerance must be finite and nonnegative")
    if max_rejections < 0:
        raise ValueError("max_rejections must be nonnegative")
    current_step = step_size
    for rejected in range(max_rejections + 1):
        fourth, fifth = _fehlberg_estimates(model, state, current_step)
        error_norm = _scaled_error(state, fourth, fifth, absolute_tolerance, relative_tolerance)
        factor = 5.0 if error_norm == 0.0 else min(5.0, max(0.2, 0.9 * error_norm ** -0.2))
        next_step = current_step * factor
        if error_norm <= 1.0:
            accepted = SimulationState(state.time + current_step, fifth, state.frame)
            return RKF45Result(accepted, current_step, next_step, error_norm, rejected)
        ####
        current_step = next_step
    ####
    raise RuntimeError("RKF45 step exceeded max_rejections")
####


def _evaluate(model: DerivativeModel, state: SimulationState) -> tuple[float, ...]:
    rates = tuple(float(value) for value in model(state))
    if len(rates) != len(state.values):
        raise ValueError("derivative dimension does not match state dimension")
    if not all(math.isfinite(value) for value in rates):
        raise ValueError("derivative values must be finite")
    return rates
####


def _advance(state: SimulationState, step_size: float, rates: tuple[float, ...]) -> SimulationState:
    values = tuple(value + step_size * rate for value, rate in zip(state.values, rates, strict=True))
    return SimulationState(state.time + step_size, values, state.frame)
####


def _fehlberg_estimates(
    model: DerivativeModel,
    state: SimulationState,
    step_size: float,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    k1 = _evaluate(model, state)
    k2 = _evaluate(model, _advance_stage(state, step_size / 4.0, step_size, k1))
    k3 = _evaluate(model, _advance_stage(state, 3.0 * step_size / 8.0, step_size, _combine_rates(((3.0 / 32.0, k1), (9.0 / 32.0, k2)))))
    k4 = _evaluate(
        model,
        _advance_stage(
            state,
            12.0 * step_size / 13.0,
            step_size,
            _combine_rates(((1932.0 / 2197.0, k1), (-7200.0 / 2197.0, k2), (7296.0 / 2197.0, k3))),
        ),
    )
    k5 = _evaluate(
        model,
        _advance_stage(
            state,
            step_size,
            step_size,
            _combine_rates(((439.0 / 216.0, k1), (-8.0, k2), (3680.0 / 513.0, k3), (-845.0 / 4104.0, k4))),
        ),
    )
    k6 = _evaluate(
        model,
        _advance_stage(
            state,
            step_size / 2.0,
            step_size,
            _combine_rates(((-8.0 / 27.0, k1), (2.0, k2), (-3544.0 / 2565.0, k3), (1859.0 / 4104.0, k4), (-11.0 / 40.0, k5))),
        ),
    )
    fourth = _combine_rates(((25.0 / 216.0, k1), (1408.0 / 2565.0, k3), (2197.0 / 4104.0, k4), (-1.0 / 5.0, k5)))
    fifth = _combine_rates(((16.0 / 135.0, k1), (6656.0 / 12825.0, k3), (28561.0 / 56430.0, k4), (-9.0 / 50.0, k5), (2.0 / 55.0, k6)))
    return tuple(state.values[index] + step_size * value for index, value in enumerate(fourth)), tuple(
        state.values[index] + step_size * value for index, value in enumerate(fifth)
    )
####


def _combine_rates(terms: tuple[tuple[float, tuple[float, ...]], ...]) -> tuple[float, ...]:
    return tuple(sum(factor * rates[index] for factor, rates in terms) for index in range(len(terms[0][1])))
####


def _advance_stage(
    state: SimulationState,
    time_offset: float,
    value_step: float,
    rates: tuple[float, ...],
) -> SimulationState:
    values = tuple(value + value_step * rate for value, rate in zip(state.values, rates, strict=True))
    return SimulationState(state.time + time_offset, values, state.frame)
####


def _scaled_error(
    state: SimulationState,
    fourth: tuple[float, ...],
    fifth: tuple[float, ...],
    absolute_tolerance: float,
    relative_tolerance: float,
) -> float:
    return max(
        abs(fifth_value - fourth_value)
        / (absolute_tolerance + relative_tolerance * max(abs(state_value), abs(fifth_value)))
        for state_value, fourth_value, fifth_value in zip(state.values, fourth, fifth, strict=True)
    )
####
