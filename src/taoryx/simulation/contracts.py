"""Small, dependency-light contracts for stepping a dynamical system."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Protocol, Sequence


@dataclass(frozen=True, slots=True)
class SimulationState:
    """A time-tagged numeric state in a named coordinate frame."""

    time: float
    values: tuple[float, ...]
    frame: str = "unspecified"


class DerivativeModel(Protocol):
    """Callable that returns state rates for the current state."""

    def __call__(self, state: SimulationState) -> Sequence[float]: ...


class Integrator(Protocol):
    """Numerical method that advances one state by one time step."""

    def step(
        self,
        model: DerivativeModel,
        state: SimulationState,
        step_size: float,
    ) -> SimulationState: ...


class TerminationReason(StrEnum):
    """Why a simulation run stopped."""

    STOP_CONDITION = "stop_condition"
    MAX_STEPS = "max_steps"


@dataclass(frozen=True, slots=True)
class SimulationResult:
    """States recorded during one run and its termination status."""

    states: tuple[SimulationState, ...]
    reason: TerminationReason


@dataclass(frozen=True, slots=True)
class EulerIntegrator:
    """Explicit Euler stepping, useful as a transparent scaffold and test oracle."""

    def step(
        self,
        model: DerivativeModel,
        state: SimulationState,
        step_size: float,
    ) -> SimulationState:
        if step_size <= 0:
            raise ValueError("step_size must be positive")
        rates = tuple(float(value) for value in model(state))
        if len(rates) != len(state.values):
            raise ValueError("derivative dimension does not match state dimension")
        values = tuple(value + step_size * rate for value, rate in zip(state.values, rates, strict=True))
        return SimulationState(state.time + step_size, values, state.frame)
