"""Shared state and callback contracts for the TAOS execution layer."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

from taoryx.contracts import Frame
from taoryx.language.expressions import ExpressionType


@dataclass(frozen=True, slots=True)
class RuntimeState:
    """A named, time-tagged state with an optional frame-aware position."""

    time: float
    values: tuple[float, ...]
    frame: Frame | str = Frame.ECFC
    named: Mapping[str, float] = field(default_factory=dict)
    value_names: tuple[str, ...] = ()

    def with_values(self, values: Sequence[float], *, time: float | None = None) -> RuntimeState:
        normalized = tuple(float(v) for v in values)
        named = dict(self.named)
        for name, value in zip(self.value_names, normalized, strict=False):
            named[name] = value
        resolved_time = self.time if time is None else time
        named["time"] = resolved_time
        return RuntimeState(resolved_time, normalized, self.frame, named, self.value_names)
    ####
####


Derivative = Callable[[RuntimeState], Sequence[float]]


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
    history: list[RuntimeState] = field(default_factory=list)
    stop_when: Callable[[RuntimeState], bool] | None = None
    events: tuple[EventCondition, ...] = ()
    integrator: str = "rk4"
    absolute_tolerance: float = 1e-8
    relative_tolerance: float = 1e-8
    max_step_size: float | None = None
    derived_definitions: Mapping[str, ExpressionType] = field(default_factory=dict)
    parameters: Mapping[str, float] = field(default_factory=dict)
    table_evaluators: Mapping[str, Callable[[Mapping[str, float]], float]] = field(default_factory=dict)
    environment_evaluator: Callable[[Mapping[str, float]], Mapping[str, float]] | None = None
    event_handlers: Mapping[str, Callable[[RuntimeState], RuntimeState]] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.step_size <= 0.0:
            raise ValueError("vehicle step_size must be positive")
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

    def active_vehicles(self) -> tuple[RuntimeVehicle, ...]:
        return tuple(vehicle for vehicle in self.vehicles.values() if vehicle.active)
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
