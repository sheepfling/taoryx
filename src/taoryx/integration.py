"""Typed numerical integration algorithms from the TAOS catalog."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .simulation.contracts import DerivativeModel, SimulationState


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
