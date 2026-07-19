"""Deterministic multi-vehicle execution contracts."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, cast

from taoryx.contracts import Frame, FrameVector3, Vector3
from taoryx.integration import IntegratorName, euler_step, normalize_integrator, rk4_step, rkf45_step, scipy_ivp_step
from taoryx.modes import DynamicsMode, Quaternion
from taoryx.rigid_body import RIGID_BODY_STATE_NAMES
from taoryx.simulation.contracts import DerivativeModel, SimulationState

from .common import Derivative, DerivativePipeline, RuntimeProblem, RuntimeState, RuntimeVehicle, SearchRestart
from .events import EventCrossing, refine_segment_final_condition
from .expressions import evaluate_definition_program

if TYPE_CHECKING:
    from .runner import RunReport


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    states: dict[str, tuple[RuntimeState, ...]]
    completed: bool
    stop_reason: str | None = None


def get_next_time_step(problem: RuntimeProblem, candidate_step: float, *, now: float | None = None) -> float:
    """Shorten a candidate step at print, table, and final-time boundaries."""

    if candidate_step <= 0.0:
        raise ValueError("candidate step must be positive")
    current = max((vehicle.state.time for vehicle in problem.active_vehicles()), default=0.0) if now is None else now
    boundaries = [boundary for boundary in problem.print_times + problem.table_knots if boundary > current]
    if problem.final_time is not None and problem.final_time > current:
        boundaries.append(problem.final_time)
    return min([candidate_step, *(boundary - current for boundary in boundaries)])
####


def integrate_active_vehicles(problem: RuntimeProblem, step: float) -> None:
    """Advance every active vehicle by one common accepted step."""

    if step <= 0.0:
        raise ValueError("integration step must be positive")
    for vehicle in problem.active_vehicles():
        _integrate_vehicle(vehicle, step)
    ####
####


def _integrate_vehicle(vehicle: RuntimeVehicle, step: float) -> None:
    """Advance one vehicle with its configured integrator and refresh hooks."""

    if step <= 0.0:
        raise ValueError("integration step must be positive")
    if vehicle.derivative is None and vehicle.point_mass_derivative is None:
        candidate = vehicle.state.with_values(vehicle.state.values, time=vehicle.state.time + step)
    else:
        start = vehicle.state
        derivative = vehicle.derivative
        point_mass_derivative = vehicle.point_mass_derivative
        if derivative is None and point_mass_derivative is None:
            raise ValueError(f"vehicle {vehicle.name} has no derivative callback")

        def model(simulation_state: SimulationState) -> tuple[float, ...]:
            staged = start.with_values(simulation_state.values, time=simulation_state.time)
            staged = _refresh_runtime_state(vehicle, staged, publish_rates=False)
            if point_mass_derivative is not None:
                return point_mass_derivative(staged.to_point_mass_state()).to_values()
            return tuple(float(value) for value in cast(Derivative, derivative)(staged))
        ####

        simulation = SimulationState(start.time, start.values, str(start.frame))
        integrator = normalize_integrator(vehicle.integrator)
        if integrator is IntegratorName.EULER:
            integrated = euler_step(cast(DerivativeModel, model), simulation, step)
            candidate = start.with_values(integrated.values, time=integrated.time)
        elif integrator is IntegratorName.RKF45:
            adaptive = rkf45_step(cast(DerivativeModel, model), simulation, step, vehicle.absolute_tolerance, vehicle.relative_tolerance)
            candidate = start.with_values(adaptive.state.values, time=adaptive.state.time)
            vehicle.step_size = min(adaptive.next_step, vehicle.max_step_size or adaptive.next_step)
        elif integrator is IntegratorName.RK4:
            integrated = rk4_step(cast(DerivativeModel, model), simulation, step)
            candidate = start.with_values(integrated.values, time=integrated.time)
        else:
            if point_mass_derivative is not None:
                raise ValueError("SciPy integrators require a generic derivative callback, not point_mass_derivative")
            candidate = start.with_values(
                scipy_ivp_step(
                    cast(DerivativeModel, model),
                    simulation,
                    step,
                    vehicle.absolute_tolerance,
                    vehicle.relative_tolerance,
                    method={
                        IntegratorName.SCIPY_RK45: "RK45",
                        IntegratorName.SCIPY_DOP853: "DOP853",
                        IntegratorName.SCIPY_RADAU: "Radau",
                        IntegratorName.SCIPY_BDF: "BDF",
                        IntegratorName.SCIPY_LSODA: "LSODA",
                    }[integrator],
                ).values,
                time=start.time + step,
            )
    candidate = _normalize_rigid_body_quaternion(vehicle, candidate)
    candidate = _refresh_runtime_state(vehicle, candidate)
    if vehicle.dynamics_mode is DynamicsMode.KINEMATIC_6DOF and vehicle.kinematic_state is not None:
        body_rate = vehicle.body_rate_provider(candidate) if vehicle.body_rate_provider is not None else Vector3(0.0, 0.0, 0.0)
        try:
            translational = candidate.to_point_mass_state()
        except ValueError:
            vehicle.kinematic_state = vehicle.kinematic_state.with_attitude_rate(body_rate, step)
        else:
            vehicle.kinematic_state = vehicle.kinematic_state.advance(
                FrameVector3(translational.position.vector, Frame.ECFC),
                FrameVector3(translational.earth_relative_velocity.vector, Frame.ECFC),
                body_rate,
                step,
            )
    vehicle.state = candidate
    vehicle.history.append(candidate)
####


def _normalize_rigid_body_quaternion(vehicle: RuntimeVehicle, state: RuntimeState) -> RuntimeState:
    """Project an accepted rigid-body state back onto unit-quaternion space."""

    if vehicle.dynamics_mode is not DynamicsMode.RIGID_BODY_6DOF:
        return state
    indices = tuple(state.value_names.index(name) for name in RIGID_BODY_STATE_NAMES[6:10])
    attitude = Quaternion(*(state.values[index] for index in indices)).normalized()
    values = list(state.values)
    for index, value in zip(indices, (attitude.w, attitude.x, attitude.y, attitude.z), strict=True):
        values[index] = value
    named = {**state.named, "qw": attitude.w, "qx": attitude.x, "qy": attitude.y, "qz": attitude.z}
    return RuntimeState(state.time, tuple(values), state.frame, named, state.value_names, state.segment_endpoints)
####


def compute_trajectories(
    problem: RuntimeProblem,
    *,
    max_steps: int = 10000,
    stop_when: Callable[[RuntimeProblem], bool] | None = None,
    synchronize_vehicles: bool = True,
) -> ExecutionResult:
    """Synchronously advance active trajectories until completion or stop."""

    if not synchronize_vehicles:
        return _compute_independent_trajectories(problem, max_steps=max_steps)

    for vehicle in problem.active_vehicles():
        vehicle.state = _refresh_runtime_state(vehicle, vehicle.state)
        vehicle.history[0] = vehicle.state
    for _ in range(max_steps):
        activate_dependent_vehicles(problem)
        if stop_when is not None and stop_when(problem):
            return ExecutionResult(_histories(problem), False, "stop_condition")
        active = problem.active_vehicles()
        if not active:
            if any(vehicle.activation_pending for vehicle in problem.vehicles.values()):
                return ExecutionResult(_histories(problem), False, "dependency_unresolved")
            return ExecutionResult(_histories(problem), True)
        stalled = tuple(vehicle for vehicle in active if vehicle.stall_detector is not None and vehicle.stall_detector(vehicle.state))
        if stalled:
            for vehicle in stalled:
                vehicle.active = False
            return ExecutionResult(_histories(problem), False, "state_stall")
        immediate_crossings = {
            vehicle.name: _current_event_crossings(vehicle)
            for vehicle in active
            if vehicle.events
        }
        if any(immediate_crossings.values()):
            if _apply_event_crossings(problem, active, immediate_crossings):
                if _has_pending_activation(problem):
                    return ExecutionResult(_histories(problem), False, "dependency_unresolved")
                return ExecutionResult(_histories(problem), True, "stop_condition")
            activate_dependent_vehicles(problem)
            continue
        step = get_next_time_step(problem, min(vehicle.step_size for vehicle in active))
        if step <= 1e-15:
            return ExecutionResult(_histories(problem), False, "boundary_stall")
        previous = {vehicle.name: vehicle.state for vehicle in active}
        integrate_active_vehicles(problem, step)
        crossings_by_vehicle = {
            vehicle.name: tuple(
                crossing
                for crossing in refine_segment_final_condition(previous[vehicle.name], vehicle.state, vehicle.events)
                if crossing.action == "stop" or crossing.name not in vehicle.fired_events
            )
            for vehicle in active
            if vehicle.events
        }
        first_crossing = min(
            ((crossing.time, vehicle_index, crossing_index, crossing) for vehicle_index, vehicle in enumerate(active) for crossing_index, crossing in enumerate(crossings_by_vehicle.get(vehicle.name, ()))),
            default=None,
        )
        if first_crossing is not None:
            restart_time = first_crossing[0]
            # A synchronized multi-vehicle step must restart every active
            # vehicle at the same physical time, not only the vehicle whose
            # condition fired.
            for vehicle in active:
                if vehicle.state.time > restart_time:
                    vehicle.state = previous[vehicle.name]
                    vehicle.history.pop()
                    restart_step = restart_time - vehicle.state.time
                    if restart_step > 1e-15:
                        _integrate_vehicle(vehicle, restart_step)
            applicable_crossings = {
                vehicle.name: tuple(
                    crossing
                    for crossing in crossings_by_vehicle.get(vehicle.name, ())
                    if abs(crossing.time - restart_time) <= 1e-9
                )
                for vehicle in active
            }
            if _apply_event_crossings(problem, active, applicable_crossings):
                if _has_pending_activation(problem):
                    return ExecutionResult(_histories(problem), False, "dependency_unresolved")
                return ExecutionResult(_histories(problem), True, "stop_condition")
        ####
        activate_dependent_vehicles(problem)
        if any(vehicle.stop_when is not None and vehicle.stop_when(vehicle.state) for vehicle in active):
            return ExecutionResult(_histories(problem), False, "stop_condition")
        if problem.final_time is not None and all(vehicle.state.time >= problem.final_time - 1e-12 for vehicle in active):
            return ExecutionResult(_histories(problem), True)
    return ExecutionResult(_histories(problem), False, "max_steps")
####


def _compute_independent_trajectories(problem: RuntimeProblem, *, max_steps: int) -> ExecutionResult:
    """Advance independent vehicles separately so one tiny step cannot throttle all.

    This mode is intentionally opt-in.  Callers must establish that vehicles do
    not exchange state during integration; the ordinary synchronized path remains
    the default for coupled trajectories.
    """

    vehicle_names = tuple(vehicle.name for vehicle in problem.active_vehicles())
    completed = True
    for name in vehicle_names:
        for vehicle in problem.vehicles.values():
            vehicle.active = vehicle.name == name
            vehicle.activation_pending = False
        result = compute_trajectories(problem, max_steps=max_steps)
        completed = completed and result.completed
        problem.vehicles[name].active = False
    return ExecutionResult(_histories(problem), completed, None if completed else "max_steps")
####


def _current_event_crossings(vehicle: RuntimeVehicle) -> tuple[EventCrossing, ...]:
    """Return events already satisfied at the vehicle's current state."""

    return tuple(
        EventCrossing(condition.name, vehicle.state.time, condition.function(vehicle.state), condition.action)
        for condition in vehicle.events
        if condition.action == "stop" or condition.name not in vehicle.fired_events
        if (condition.predicate(vehicle.state) if condition.predicate is not None else condition.function(vehicle.state) >= 0.0)
    )
####


def _apply_event_crossings(
    problem: RuntimeProblem,
    active: Sequence[RuntimeVehicle],
    crossings_by_vehicle: Mapping[str, Sequence[EventCrossing]],
) -> bool:
    """Apply event handlers in vehicle and source order; return when all stop."""

    for vehicle in active:
        for crossing in crossings_by_vehicle.get(vehicle.name, ()):
            if crossing.action != "stop" and crossing.name in vehicle.fired_events:
                continue
            segment_from = vehicle.segment_number
            handler = vehicle.event_handlers.get(crossing.name)
            if handler is not None:
                vehicle.state = handler(vehicle.state)
                vehicle.state = _refresh_runtime_state(vehicle, vehicle.state)
                vehicle.history[-1] = vehicle.state
            vehicle.fired_events.add(crossing.name)
            condition = next((item for item in vehicle.events if item.name == crossing.name), None)
            problem.event_history.append(
                {
                    "name": crossing.name,
                    "vehicle": vehicle.name,
                    "time": crossing.time,
                    "action": crossing.action,
                    "signal": condition.signal if condition is not None and condition.signal is not None else crossing.name,
                    "residual": crossing.residual,
                    "segment_from": segment_from,
                    "segment_to": vehicle.segment_number,
                    "source": condition.source if condition is not None else None,
                }
            )
            if crossing.action == "stop":
                vehicle.active = False
                vehicle.activation_pending = False
                break
    return not any(vehicle.active for vehicle in active)
####


def run_taos(
    problem: RuntimeProblem | str | Path,
    table_paths: Sequence[str | Path] = (),
    *,
    output_dir: str | Path = ".",
    max_steps: int = 10000,
    integrator: str | None = None,
) -> ExecutionResult | RunReport:
    """Run a resolved graph or ingest and execute a TAOS problem file.

    The graph form remains available for low-level callers. File inputs are
    delegated to the same runner used by the command-line interface.
    """

    if isinstance(problem, RuntimeProblem):
        if table_paths:
            raise ValueError("table paths are only valid when running a problem file")
        if integrator is not None:
            selected = normalize_integrator(integrator).value
            for vehicle in problem.vehicles.values():
                vehicle.integrator = selected
        return compute_trajectories(problem, max_steps=max_steps)
    from .runner import run_files

    return run_files(problem, tuple(table_paths), output_dir=output_dir, max_steps=max_steps, integrator=integrator)
####


def dispatch_search_and_restart(problem: RuntimeProblem, restarts: Iterable[SearchRestart]) -> float | None:
    """Apply parameter updates and return the earliest invalidated time."""

    parameters = problem.metadata.setdefault("parameters", {})
    if not isinstance(parameters, dict):
        raise TypeError("problem parameter store must be a dictionary")
    updates = tuple(restarts)
    for restart in updates:
        parameters[restart.parameter] = restart.value
    return min((restart.restart_time for restart in updates), default=None)
####


def activate_dependent_vehicles(problem: RuntimeProblem) -> tuple[str, ...]:
    """Activate pending vehicles once dependency histories are available."""

    activated: list[str] = []
    for vehicle in problem.vehicles.values():
        if vehicle.active or not vehicle.activation_pending or not all(problem.vehicles[name].history for name in vehicle.dependencies):
            continue
        if any(problem.vehicles[name].segment_number < required for name, required in vehicle.dependency_segments.items()):
            continue
        if vehicle.dependencies:
            source = problem.vehicles[vehicle.dependencies[0]]
            names = vehicle.state.value_names
            named = dict(source.state.named)
            values = tuple(named.get(name, vehicle.state.named.get(name, 0.0)) for name in names)
            vehicle.state = RuntimeState(source.state.time, values, source.state.frame, named, names, source.state.segment_endpoints)
            if vehicle.activation_handler is not None:
                vehicle.state = vehicle.activation_handler(vehicle.state)
            vehicle.history[:] = [vehicle.state]
            vehicle.active = True
            vehicle.activation_pending = False
            activated.append(vehicle.name)
    return tuple(activated)
####


def compute_derivatives(initial: dict[str, float], pipeline: DerivativePipeline) -> dict[str, float]:
    """Execute the ordered derivative pipeline."""

    return pipeline.evaluate(initial)
####


def _histories(problem: RuntimeProblem) -> dict[str, tuple[RuntimeState, ...]]:
    return {name: tuple(vehicle.history) for name, vehicle in problem.vehicles.items()}
####


def _has_pending_activation(problem: RuntimeProblem) -> bool:
    """Report dependent vehicles that still await an unreachable source segment."""

    return any(vehicle.activation_pending for vehicle in problem.vehicles.values())
####


def _refresh_runtime_state(vehicle: RuntimeVehicle, state: RuntimeState, *, publish_rates: bool = True) -> RuntimeState:
    """Refresh environment and user-defined values at every derivative stage."""

    named = dict(state.named)
    named.update(vehicle.control_values)
    if vehicle.environment_evaluator is not None:
        environment_values = named
        if not publish_rates:
            environment_values = {**named, "_runtime_derivative_stage": 1.0}
        named.update(vehicle.environment_evaluator(environment_values))
    if vehicle.derived_definitions:
        named = evaluate_definition_program(
            vehicle.derived_definitions,
            {key: value for key, value in named.items() if key not in vehicle.derived_definitions},
            parameters=vehicle.parameters,
            table_evaluators=vehicle.table_evaluators,
        )
    if vehicle.definition_evaluator is not None:
        named.update(vehicle.definition_evaluator(named))
    named["time"] = state.time
    refreshed = RuntimeState(state.time, state.values, state.frame, named, state.value_names, state.segment_endpoints)
    if publish_rates and vehicle.publish_derived_rates and vehicle.derivative is not None:
        try:
            rates = tuple(float(value) for value in vehicle.derivative(refreshed))
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            rates = ()
        if len(rates) == len(state.value_names):
            for name, rate in zip(state.value_names, rates, strict=True):
                if not name.endswith("dt"):
                    named[f"{name}dt"] = rate
    return RuntimeState(state.time, state.values, state.frame, named, state.value_names, state.segment_endpoints)
