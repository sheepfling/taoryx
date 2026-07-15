"""Shared state and callback contracts for the TAOS execution layer."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from copy import copy, deepcopy
from dataclasses import dataclass, field

from taoryx.contracts import Frame, Vector3
from taoryx.language.expressions import ExpressionType
from taoryx.modes import DynamicsMode, Kinematic6DofState
from taoryx.outputs import VehicleKind
from taoryx.state import PointMassRates, PointMassState


@dataclass(frozen=True, slots=True)
class RuntimeState:
    """A named, time-tagged state with an optional frame-aware position."""

    time: float
    values: tuple[float, ...]
    frame: Frame | str = Frame.ECFC
    named: Mapping[str, float] = field(default_factory=dict)
    value_names: tuple[str, ...] = ()
    segment_endpoints: Mapping[int, RuntimeState] = field(default_factory=dict)

    def with_values(self, values: Sequence[float], *, time: float | None = None) -> RuntimeState:
        normalized = tuple(float(v) for v in values)
        named = dict(self.named)
        for name, value in zip(self.value_names, normalized, strict=False):
            named[name] = value
        resolved_time = self.time if time is None else time
        named["time"] = resolved_time
        return RuntimeState(resolved_time, normalized, self.frame, named, self.value_names, self.segment_endpoints)
    ####

    def interpolated(self, other: RuntimeState, time: float) -> RuntimeState:
        """Linearly interpolate this state and its numeric named observables."""

        if other.time == self.time:
            if time != self.time:
                raise ValueError("cannot interpolate distinct times from a zero-duration state")
            return self
        fraction = (time - self.time) / (other.time - self.time)
        if fraction < -1e-12 or fraction > 1.0 + 1e-12:
            raise ValueError("interpolation time lies outside the state interval")
        fraction = min(1.0, max(0.0, fraction))
        values = tuple(left + fraction * (right - left) for left, right in zip(self.values, other.values, strict=True))
        named = dict(self.named)
        for name in set(self.named) | set(other.named):
            left = self.named.get(name)
            right = other.named.get(name)
            if isinstance(left, (int, float)) and isinstance(right, (int, float)):
                named[name] = float(left) + fraction * (float(right) - float(left))
            elif right is not None:
                named[name] = right
        return RuntimeState(time, values, self.frame, named, self.value_names, self.segment_endpoints)
    ####

    def to_point_mass_state(self) -> PointMassState:
        """Convert the canonical nine-value runtime state to physical state.

        Runtime lowering may carry many named derived values, but only the
        canonical TAOS integration names cross into the point-mass kernel.
        """

        expected = PointMassState.STATE_NAMES
        if self.value_names and self.value_names != expected:
            raise ValueError("runtime state names do not match the canonical TAOS point-mass order")
        return PointMassState.from_values(self.time, list(self.values))
        ####

    @classmethod
    def from_point_mass_state(cls, state: PointMassState) -> RuntimeState:
        """Wrap a validated physical state for runtime scheduling and output."""

        values = state.to_values()
        named = dict(zip(PointMassState.STATE_NAMES, values, strict=True))
        named["time"] = state.time
        return cls(state.time, values, Frame.ECFC, named, PointMassState.STATE_NAMES)
        ####
####


Derivative = Callable[[RuntimeState], Sequence[float]]
PointMassDerivative = Callable[[PointMassState], PointMassRates]
BodyRateProvider = Callable[[RuntimeState], Vector3]
StallDetector = Callable[[RuntimeState], bool]


@dataclass(slots=True)
class RuntimeVehicle:
    """Mutable integration record for one vehicle trajectory."""

    name: str
    state: RuntimeState
    derivative: Derivative | None = None
    step_size: float = 1.0
    dependencies: tuple[str, ...] = ()
    dependency_segments: Mapping[str, int] = field(default_factory=dict)
    segment_number: int = 1
    active: bool = True
    activation_pending: bool = True
    history: list[RuntimeState] = field(default_factory=list)
    stop_when: Callable[[RuntimeState], bool] | None = None
    events: tuple[EventCondition, ...] = ()
    integrator: str = "rk4"
    absolute_tolerance: float = 1e-8
    relative_tolerance: float = 1e-8
    max_step_size: float | None = None
    derived_definitions: Mapping[str, ExpressionType] = field(default_factory=dict)
    definition_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None
    parameters: Mapping[str, float] = field(default_factory=dict)
    control_values: Mapping[str, float] = field(default_factory=dict)
    table_evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] = field(default_factory=dict)
    environment_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None
    event_handlers: Mapping[str, Callable[[RuntimeState], RuntimeState]] = field(default_factory=dict)
    activation_handler: Callable[[RuntimeState], RuntimeState] | None = None
    point_mass_derivative: PointMassDerivative | None = None
    dynamics_mode: DynamicsMode = DynamicsMode.POINT_MASS
    publish_derived_rates: bool = True
    kinematic_state: Kinematic6DofState | None = None
    body_rate_provider: BodyRateProvider | None = None
    stall_detector: StallDetector | None = None
    vehicle_kind: VehicleKind = VehicleKind.GENERIC
    fired_events: set[str] = field(default_factory=set)

    def __post_init__(self) -> None:
        if self.step_size <= 0.0:
            raise ValueError("vehicle step_size must be positive")
        if self.active:
            self.activation_pending = False
        if self.control_values:
            named = {**self.state.named, **self.control_values}
            self.state = RuntimeState(self.state.time, self.state.values, self.state.frame, named, self.state.value_names, self.state.segment_endpoints)
        if self.dynamics_mode is DynamicsMode.KINEMATIC_6DOF and self.kinematic_state is None:
            raise ValueError("kinematic-6dof vehicles require a kinematic state sidecar")
        if self.dynamics_mode is not DynamicsMode.KINEMATIC_6DOF and self.kinematic_state is not None:
            raise ValueError("kinematic state sidecars require kinematic-6dof mode")
        if not self.history:
            self.history.append(self.state)
        ####
    ####


@dataclass(slots=True)
class RuntimeProblem:
    """Resolved vehicle graph and problem-level execution settings."""

    vehicles: dict[str, RuntimeVehicle]
    print_times: tuple[float, ...] = ()
    table_knots: tuple[float, ...] = ()
    final_time: float | None = None
    metadata: dict[str, object] = field(default_factory=dict)
    event_history: list[dict[str, object]] = field(default_factory=list)

    def active_vehicles(self) -> tuple[RuntimeVehicle, ...]:
        return tuple(vehicle for vehicle in self.vehicles.values() if vehicle.active)
    ####

    def observe(self, vehicle: str | None = None, *, status_names: Sequence[str] = (), include_deep: bool = False) -> object:
        """Return a tiered observation for one vehicle or all vehicles."""

        from .observations import observe_vehicle

        if vehicle is not None:
            return observe_vehicle(self.vehicles[vehicle], status_names=status_names, include_deep=include_deep)
        return {name: observe_vehicle(item, status_names=status_names, include_deep=include_deep) for name, item in self.vehicles.items()}
    ####

    def clone_at(self, time: float, *, resume: bool = True) -> RuntimeProblem:
        """Clone the executable graph at a recorded or interpolated flight time.

        Vehicle callbacks remain shared because they are executable model code;
        mutable vehicle state, histories, metadata, and event sets are copied so
        the returned graph can be advanced independently.
        """

        if not self.vehicles:
            raise ValueError("cannot clone an empty runtime problem")
        histories: dict[str, list[RuntimeState]] = {}
        cloned_vehicles: dict[str, RuntimeVehicle] = {}
        for name, source in self.vehicles.items():
            if not source.history:
                raise ValueError(f"vehicle {name!r} has no history to clone")
            first = source.history[0].time
            last = source.history[-1].time
            if time < first - 1e-12 or time > last + 1e-12:
                raise ValueError(f"clone time {time} is outside vehicle {name!r} history [{first}, {last}]")
            bounded_time = min(last, max(first, time))
            retained = [state for state in source.history if state.time < bounded_time - 1e-12]
            exact = next((state for state in source.history if abs(state.time - bounded_time) <= 1e-12), None)
            if exact is None:
                upper_index = next(index for index, state in enumerate(source.history) if state.time > bounded_time)
                exact = source.history[upper_index - 1].interpolated(source.history[upper_index], bounded_time)
            retained.append(exact)
            histories[name] = retained
            clone = copy(source)
            clone.state = exact
            clone.history = retained
            clone.fired_events = set(event for event in source.fired_events if event in {condition.name for condition in source.events if condition.action != "stop"})
            if resume:
                clone.active = True
                clone.activation_pending = False
            cloned_vehicles[name] = clone
        cloned = RuntimeProblem(
            cloned_vehicles,
            self.print_times,
            self.table_knots,
            self.final_time,
            deepcopy(self.metadata),
            deepcopy(self.event_history),
        )
        cloned.metadata["cloned_at_time"] = float(time)
        return cloned
    ####
####


@dataclass(frozen=True, slots=True)
class StepBoundary:
    """A future time at which a step must end."""

    time: float
    reason: str


@dataclass(frozen=True, slots=True)
class EventCondition:
    """A scalar event residual evaluated against a runtime state."""

    name: str
    function: Callable[[RuntimeState], float]
    action: str = "stop"
    predicate: Callable[[RuntimeState], bool] | None = None
    signal: str | None = None
    source: str | None = None


@dataclass(frozen=True, slots=True)
class SearchRestart:
    """A parameter update and earliest history time it invalidates."""

    parameter: str
    value: float
    restart_time: float


@dataclass(frozen=True, slots=True)
class DerivativePipeline:
    """Ordered derivative stages; each stage receives and returns named state."""

    stages: tuple[Callable[[Mapping[str, float]], Mapping[str, float]], ...]

    def evaluate(self, initial: Mapping[str, float]) -> dict[str, float]:
        current = dict(initial)
        for stage in self.stages:
            current = dict(stage(current))
        ####
        return current
    ####
####
