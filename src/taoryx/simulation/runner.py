"""Execution loop for the generic simulation kernel."""

from __future__ import annotations

from collections.abc import Callable

from .contracts import (
    DerivativeModel,
    EulerIntegrator,
    Integrator,
    SimulationResult,
    SimulationState,
    TerminationReason,
)


class SimulationRunner:
    """Advance a model while retaining a complete state history."""

    def __init__(self, integrator: Integrator | None = None) -> None:
        self.integrator = integrator or EulerIntegrator()

    def run(
        self,
        initial: SimulationState,
        model: DerivativeModel,
        *,
        step_size: float,
        max_steps: int,
        stop_when: Callable[[SimulationState], bool] | None = None,
    ) -> SimulationResult:
        if step_size <= 0:
            raise ValueError("step_size must be positive")
        if max_steps <= 0:
            raise ValueError("max_steps must be positive")

        states = [initial]
        current = initial
        for _ in range(max_steps):
            if stop_when is not None and stop_when(current):
                return SimulationResult(tuple(states), TerminationReason.STOP_CONDITION)
            current = self.integrator.step(model, current, step_size)
            states.append(current)
            if stop_when is not None and stop_when(current):
                return SimulationResult(tuple(states), TerminationReason.STOP_CONDITION)
        return SimulationResult(tuple(states), TerminationReason.MAX_STEPS)
