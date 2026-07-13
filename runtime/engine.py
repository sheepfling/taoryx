"""Deterministic multi-vehicle execution contracts."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass

from .common import DerivativePipeline, RuntimeProblem, RuntimeState, RuntimeVehicle, SearchRestart, StepBoundary


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    states: dict[str, tuple[RuntimeState, ...]]
    completed: bool
    stop_reason: str | None = None


def get_next_time_step(problem: RuntimeProblem, candidate_step: float, *, now: float | None = None) -> float:
    """Shorten a candidate step at vehicle, print, table, and final-time boundaries."""

    if candidate_step <= 0.0:
        raise ValueError("candidate step must be positive")
    current = max((vehicle.state.time for vehicle in problem.active_vehicles()), default=0.0) if now is None else now
    boundaries = [boundary for boundary in problem.print_times + problem.table_knots if boundary > current]
    if problem.final_time is not None and problem.final_time > current:
        boundaries.append(problem.final_time)
    return min([candidate_step, *(boundary - current for boundary in boundaries)])
####


def integrate_active_vehicles(problem: RuntimeProblem, step: float) -> None:
    """Advance every active vehicle by a common accepted step."""

    if step <= 0.0:
        raise ValueError("integration step must be positive")
    for vehicle in problem.active_vehicles():
        if vehicle.derivative is None:
            vehicle.state = vehicle.state.with_values(vehicle.state.values, time=vehicle.state.time + step)
        else:
            rates = tuple(float(value) for value in vehicle.derivative(vehicle.state))
            if len(rates) != len(vehicle.state.values):
                raise ValueError(f"derivative dimension mismatch for {vehicle.name}")
            vehicle.state = RuntimeState(vehicle.state.time + step, tuple(value + step * rate for value, rate in zip(vehicle.state.values, rates, strict=True)), vehicle.state.frame, vehicle.state.named)
        vehicle.history.append(vehicle.state)
    ####
####


def compute_trajectories(problem: RuntimeProblem, *, max_steps: int = 10000, stop_when: Callable[[RuntimeProblem], bool] | None = None) -> ExecutionResult:
    """Synchronously advance active trajectories until completion or a stop callback."""

    for _ in range(max_steps):
        if stop_when is not None and stop_when(problem):
            return ExecutionResult(_histories(problem), False, "stop_condition")
        active = problem.active_vehicles()
        if not active:
            return ExecutionResult(_histories(problem), True)
        candidate = min(vehicle.step_size for vehicle in active)
        step = get_next_time_step(problem, candidate)
        if step <= 1e-15:
            return ExecutionResult(_histories(problem), False, "boundary_stall")
        integrate_active_vehicles(problem, step)
        if problem.final_time is not None and all(vehicle.state.time >= problem.final_time - 1e-12 for vehicle in active):
            return ExecutionResult(_histories(problem), True)
    return ExecutionResult(_histories(problem), False, "max_steps")
####


def run_taos(problem: RuntimeProblem, *, max_steps: int = 10000) -> ExecutionResult:
    """Run an already-resolved problem graph; file/AST ingestion stays at the parser boundary."""

    return compute_trajectories(problem, max_steps=max_steps)
####


def dispatch_search_and_restart(problem: RuntimeProblem, restarts: Iterable[SearchRestart]) -> float | None:
    """Apply parameter updates and return the earliest required restart time."""

    ordered = tuple(restarts)
    parameters = problem.metadata.setdefault("parameters", {})
    if not isinstance(parameters, dict):
        raise TypeError("problem parameter store must be a dictionary")
    for restart in ordered:
        parameters[restart.parameter] = restart.value
    return min((restart.restart_time for restart in ordered), default=None)
####


def activate_dependent_vehicles(problem: RuntimeProblem) -> tuple[str, ...]:
    """Activate pending vehicles once every dependency has a history sample."""

    activated: list[str] = []
    for vehicle in problem.vehicles.values():
        if vehicle.active or all(problem.vehicles[name].history for name in vehicle.dependencies):
            if not vehicle.active:
                vehicle.active = True
                activated.append(vehicle.name)
    return tuple(activated)
####


def compute_derivatives(initial: dict[str, float], pipeline: DerivativePipeline) -> dict[str, float]:
    """Execute the ordered environment/force/guidance/output derivative pipeline."""

    return pipeline.evaluate(initial)
####


def _histories(problem: RuntimeProblem) -> dict[str, tuple[RuntimeState, ...]]:
    return {name: tuple(vehicle.history) for name, vehicle in problem.vehicles.items()}
####
