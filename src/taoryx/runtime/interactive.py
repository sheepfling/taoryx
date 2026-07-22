"""Deterministic, externally driven runtime stepping.

This layer deliberately reuses :mod:`taoryx.runtime.engine`. It adds lifecycle
and command-stream semantics around the existing typed state and integrator
contracts; it is not a second numerical kernel.
"""

from __future__ import annotations

import json
import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING

from taoryx.control import SegmentController, VehicleObservation

from .common import Derivative, RuntimeProblem, RuntimeState
from .engine import integrate_active_vehicles

if TYPE_CHECKING:
    from taoryx.outputs import RunArtifact


class InteractiveStatus(StrEnum):
    CREATED = "created"
    RUNNING = "running"
    PAUSED = "paused"
    INTERRUPTED = "interrupted"
    COMPLETED = "completed"
    FAILED = "failed"
####


class EventAction(StrEnum):
    """Deterministic actions available at an accepted event boundary."""

    STOP = "stop"
    TRANSITION = "transition"
    SIGNAL = "signal"
####


@dataclass(frozen=True, slots=True)
class ControlSpec:
    """Bounds and units for one named external command channel."""

    name: str
    unit: str | None = None
    default: float = 0.0
    lower: float = -math.inf
    upper: float = math.inf
    slew_rate: float | None = None
    modes: tuple[str, ...] = ("point-mass", "kinematic-6dof", "rigid-body-6dof")

    def __post_init__(self) -> None:
        if not self.name or not self.name.strip():
            raise ValueError("control name must not be empty")
        if not math.isfinite(self.default) or math.isnan(self.lower) or math.isnan(self.upper):
            raise ValueError("control default must be finite and bounds must not be NaN")
        if self.lower > self.upper or not self.lower <= self.default <= self.upper:
            raise ValueError("control default must lie within ordered bounds")
        if self.slew_rate is not None and (not math.isfinite(self.slew_rate) or self.slew_rate < 0.0):
            raise ValueError("control slew_rate must be finite and nonnegative")
        ####
    ####


@dataclass(frozen=True, slots=True)
class AppliedCommand:
    name: str
    requested: float
    applied: float
    unit: str | None
    clamped: bool = False
####


@dataclass(frozen=True, slots=True)
class StatusSpec:
    """A named observable status channel exposed by a runtime state."""

    name: str
    source: str | None = None
    unit: str | None = None
    modes: tuple[str, ...] = ("point-mass", "kinematic-6dof", "rigid-body-6dof")

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("status name must not be empty")
        if self.source is not None and not self.source.strip():
            raise ValueError("status source must not be empty")
        ####
    ####


@dataclass(frozen=True, slots=True)
class OutputSubscription:
    """A renderer-neutral request for semantic runtime channels."""

    channels: tuple[str, ...] = ()
    sample_interval: float | None = None
    include_events: bool = True

    def __post_init__(self) -> None:
        if self.sample_interval is not None and (not math.isfinite(self.sample_interval) or self.sample_interval <= 0.0):
            raise ValueError("output sample_interval must be positive and finite")
        ####
    ####


EventPredicate = Callable[[RuntimeState], bool]


@dataclass(frozen=True, slots=True)
class EventSpec:
    """A runtime event rule evaluated after a numerical boundary."""

    name: str
    predicate: EventPredicate
    action: EventAction = EventAction.SIGNAL
    signal: str | None = None
    once: bool = True
    source: str | None = None

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("event name must not be empty")
        if self.action is EventAction.SIGNAL and not (self.signal or self.name).strip():
            raise ValueError("signal events require a signal name")
        ####
    ####


@dataclass(frozen=True, slots=True)
class RuntimeEvent:
    """Structured record emitted when an event rule is accepted."""

    name: str
    vehicle: str
    time: float
    action: EventAction
    signal: str | None = None
    source: str | None = None
    segment_from: int | None = None
    segment_to: int | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "vehicle": self.vehicle,
            "time": self.time,
            "action": self.action.value,
            "signal": self.signal,
            "source": self.source,
            "segment_from": self.segment_from,
            "segment_to": self.segment_to,
        }
    ####
####


@dataclass(frozen=True, slots=True)
class ReplayFrame:
    duration: float
    commands: Mapping[str, float]
####


@dataclass(frozen=True, slots=True)
class InteractiveSnapshot:
    """One accepted interactive step and its observable effects."""

    time_start: float
    time_end: float
    states: Mapping[str, RuntimeState]
    commands: tuple[AppliedCommand, ...]
    events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()
    status: InteractiveStatus = InteractiveStatus.RUNNING
    runtime_events: tuple[RuntimeEvent, ...] = ()
    statuses: Mapping[str, Mapping[str, float]] = field(default_factory=dict)

    def as_dict(self) -> dict[str, object]:
        return {
            "time_start": self.time_start,
            "time_end": self.time_end,
            "status": self.status.value,
            "commands": [
                {
                    "name": command.name,
                    "requested": command.requested,
                    "applied": command.applied,
                    "unit": command.unit,
                    "clamped": command.clamped,
                }
                for command in self.commands
            ],
            "events": list(self.events),
            "runtime_events": [event.as_dict() for event in self.runtime_events],
            "diagnostics": list(self.diagnostics),
            "statuses": {vehicle: dict(values) for vehicle, values in self.statuses.items()},
            "states": {
                name: {
                    "time": state.time,
                    "values": list(state.values),
                    "frame": str(state.frame),
                    "named": dict(state.named),
                }
                for name, state in self.states.items()
            },
        }
    ####
####


@dataclass(frozen=True, slots=True)
class InteractiveArtifact:
    """Replayable normalized output for an interactive session."""

    schema_version: int
    status: InteractiveStatus
    snapshots: tuple[InteractiveSnapshot, ...]

    def write_json(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "status": self.status.value,
            "snapshots": [snapshot.as_dict() for snapshot in self.snapshots],
        }
        destination.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        return destination
    ####
####


ControlModel = Callable[[str, RuntimeState, Mapping[str, float]], Mapping[str, float]]


@dataclass(slots=True)
class InteractiveSession:
    """Externally driven deterministic session over a resolved runtime problem."""

    problem: RuntimeProblem
    controls: tuple[ControlSpec, ...] = ()
    control_model: ControlModel | None = None
    segment_controllers: Mapping[str, SegmentController] = field(default_factory=dict)
    status: InteractiveStatus = InteractiveStatus.CREATED
    snapshots: list[InteractiveSnapshot] = field(default_factory=list)
    command_history: list[ReplayFrame] = field(default_factory=list)
    status_specs: tuple[StatusSpec, ...] = ()
    event_specs: tuple[EventSpec, ...] = ()
    output_subscriptions: tuple[OutputSubscription, ...] = ()
    event_history: list[RuntimeEvent] = field(default_factory=list)
    _last_commands: dict[str, float] = field(init=False, repr=False)
    _fired_events: set[tuple[str, str]] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        names = [control.name for control in self.controls]
        if len(set(names)) != len(names):
            raise ValueError("interactive control names must be unique")
        for control in self.controls:
            for vehicle in self.problem.vehicles.values():
                if vehicle.dynamics_mode.value not in control.modes:
                    raise ValueError(f"control {control.name!r} is not valid for {vehicle.dynamics_mode.value}")
        status_names = [status.name for status in self.status_specs]
        if len(set(status_names)) != len(status_names):
            raise ValueError("interactive status names must be unique")
        event_names = [event.name for event in self.event_specs]
        if len(set(event_names)) != len(event_names):
            raise ValueError("interactive event names must be unique")
        unknown_controllers = sorted(set(self.segment_controllers) - set(self.problem.vehicles))
        if unknown_controllers:
            raise ValueError(f"segment controller references unknown vehicle(s): {', '.join(unknown_controllers)}")
        ####
        self._last_commands = {control.name: control.default for control in self.controls}
        self._fired_events = set()
        for vehicle in self.problem.vehicles.values():
            if vehicle.derivative is None:
                continue
            original = vehicle.derivative

            def commanded(state: RuntimeState, *, vehicle_name: str = vehicle.name, derivative: Derivative = original) -> Sequence[float]:
                values = dict(state.named)
                values.update(self._last_commands)
                if self.control_model is not None:
                    values.update(self.control_model(vehicle_name, state, self._last_commands))
                commanded_state = RuntimeState(state.time, state.values, state.frame, values, state.value_names, state.segment_endpoints)
                return derivative(commanded_state)

            vehicle.derivative = commanded
        ####
    ####

    @property
    def time(self) -> float:
        return max((vehicle.state.time for vehicle in self.problem.vehicles.values()), default=0.0)
    ####

    def _normalize_commands(self, requested: Mapping[str, float], duration: float) -> tuple[AppliedCommand, ...]:
        known = {control.name: control for control in self.controls}
        unknown = sorted(set(requested) - set(known))
        if unknown:
            raise ValueError(f"unknown interactive control(s): {', '.join(unknown)}")
        applied: list[AppliedCommand] = []
        for control in self.controls:
            value = float(requested.get(control.name, self._last_commands[control.name]))
            if not math.isfinite(value):
                raise ValueError(f"interactive control {control.name!r} must be finite")
            bounded = min(control.upper, max(control.lower, value))
            if control.slew_rate is not None:
                maximum_delta = control.slew_rate * duration
                previous = self._last_commands[control.name]
                bounded = min(previous + maximum_delta, max(previous - maximum_delta, bounded))
            applied.append(AppliedCommand(control.name, value, bounded, control.unit, bounded != value))
        self._last_commands = {command.name: command.applied for command in applied}
        return tuple(applied)
    ####

    def step(self, duration: float, commands: Mapping[str, float] | None = None) -> InteractiveSnapshot:
        """Advance exactly ``duration`` using a boundary-applied command set."""

        if self.status in {InteractiveStatus.INTERRUPTED, InteractiveStatus.COMPLETED, InteractiveStatus.FAILED}:
            raise RuntimeError(f"cannot step an interactive session in {self.status.value} state")
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("interactive step duration must be positive and finite")
        start = self.time
        controller_commands: dict[str, float] = {}
        controller_diagnostics: list[str] = []
        for vehicle_name, controller in self.segment_controllers.items():
            vehicle = self.problem.vehicles[vehicle_name]
            observed_state = {
                **dict(zip(vehicle.state.value_names, vehicle.state.values, strict=False)),
                **vehicle.state.named,
            }
            observation = VehicleObservation(vehicle.state.time, observed_state)
            generated = controller.step(observation, duration)
            controller_commands.update(generated.values)
            if generated.saturated:
                controller_diagnostics.append(f"controller-saturated:{vehicle_name}:{','.join(generated.saturated)}")
        requested_commands = {**controller_commands, **(commands or {})}
        applied = self._normalize_commands(requested_commands, duration)
        self.status = InteractiveStatus.RUNNING
        try:
            integrate_active_vehicles(self.problem, duration)
            events: list[str] = []
            for vehicle in self.problem.active_vehicles():
                for event in vehicle.events:
                    if event.predicate is not None and event.predicate(vehicle.state):
                        events.append(f"{vehicle.name}:{event.name}")
                        if event.action == "stop":
                            vehicle.active = False
            runtime_events: list[RuntimeEvent] = []
            for vehicle in self.problem.vehicles.values():
                for spec in self.event_specs:
                    key = (vehicle.name, spec.name)
                    if spec.once and key in self._fired_events:
                        continue
                    if not spec.predicate(vehicle.state):
                        continue
                    runtime_event = RuntimeEvent(
                        spec.name,
                        vehicle.name,
                        vehicle.state.time,
                        spec.action,
                        spec.signal or spec.name,
                        spec.source,
                        vehicle.segment_number,
                        vehicle.segment_number,
                    )
                    runtime_events.append(runtime_event)
                    self.event_history.append(runtime_event)
                    self._fired_events.add(key)
                    if spec.action is EventAction.STOP:
                        vehicle.active = False
            if events:
                self.status = InteractiveStatus.COMPLETED
            if any(event.action is EventAction.STOP for event in runtime_events):
                self.status = InteractiveStatus.COMPLETED
        except Exception:
            self.status = InteractiveStatus.FAILED
            raise
        # Replay the requested stream, not only the bounded result. The
        # limiter must be re-applied so the replay verifies control semantics.
        self.command_history.append(ReplayFrame(duration, dict(requested_commands)))
        statuses = {
            vehicle.name: {
                status.name: self._last_commands.get(
                    status.source or status.name,
                    vehicle.state.named.get(status.source or status.name, 0.0),
                )
                for status in self.status_specs
                if vehicle.dynamics_mode.value in status.modes
            }
            for vehicle in self.problem.vehicles.values()
        }
        snapshot = InteractiveSnapshot(
            start,
            self.time,
            {name: vehicle.state for name, vehicle in self.problem.vehicles.items()},
            applied,
            tuple(events) + tuple(event.name for event in runtime_events),
            diagnostics=tuple(controller_diagnostics),
            status=self.status,
            runtime_events=tuple(runtime_events),
            statuses=statuses,
        )
        self.snapshots.append(snapshot)
        return snapshot
    ####

    def pause(self) -> None:
        if self.status not in {InteractiveStatus.CREATED, InteractiveStatus.RUNNING}:
            raise RuntimeError(f"cannot pause an interactive session in {self.status.value} state")
        self.status = InteractiveStatus.PAUSED
        ####

    def resume(self) -> None:
        if self.status is not InteractiveStatus.PAUSED:
            raise RuntimeError(f"cannot resume an interactive session in {self.status.value} state")
        self.status = InteractiveStatus.RUNNING
        ####

    def interrupt(self) -> None:
        if self.status in {InteractiveStatus.COMPLETED, InteractiveStatus.FAILED}:
            return
        self.status = InteractiveStatus.INTERRUPTED
        ####

    def artifact(self) -> InteractiveArtifact:
        return InteractiveArtifact(1, self.status, tuple(self.snapshots))
    ####

    def to_run_artifact(self) -> RunArtifact:
        """Project the session history into the batch telemetry contract."""

        from taoryx.outputs import DynamicsKind, RunArtifact, TelemetryChannel, VehicleTelemetry

        dynamics = {
            "point-mass": DynamicsKind.POINT_MASS_3DOF,
            "kinematic-6dof": DynamicsKind.KINEMATIC_3_PLUS_3_DOF,
            "rigid-body-6dof": DynamicsKind.RIGID_BODY_6DOF,
        }
        vehicles: dict[str, VehicleTelemetry] = {}
        for name, vehicle in self.problem.vehicles.items():
            history = tuple(vehicle.history)
            times = [state.time for state in history]
            value_names = vehicle.state.value_names or tuple(f"state_{index}" for index in range(len(vehicle.state.values)))
            channels = {
                value_name: TelemetryChannel(
                    source_name=value_name,
                    semantic_name=value_name,
                    values=[state.values[index] for state in history],
                )
                for index, value_name in enumerate(value_names)
            }
            vehicles[name] = VehicleTelemetry(
                vehicle_id=name,
                name=name,
                kind=vehicle.vehicle_kind,
                dynamics=dynamics[vehicle.dynamics_mode.value],
                attitude_source="interactive-controller" if vehicle.kinematic_state is not None else None,
                times=times,
                channels=channels,
            )
        return RunArtifact(
            problem="interactive-session",
            vehicles=vehicles,
            commands=[{"duration": frame.duration, "commands": dict(frame.commands)} for frame in self.command_history],
            events=[event.as_dict() for event in self.event_history],
            visualization={
                "source": "InteractiveSession",
                "controls": [
                    {
                        "name": control.name,
                        "unit": control.unit,
                        "default": control.default,
                        "lower": control.lower,
                        "upper": control.upper,
                        "slew_rate": control.slew_rate,
                        "modes": list(control.modes),
                    }
                    for control in self.controls
                ],
                "statuses": [
                    {"name": status.name, "source": status.source, "unit": status.unit, "modes": list(status.modes)}
                    for status in self.status_specs
                ],
                "subscriptions": [
                    {"channels": list(subscription.channels), "sample_interval": subscription.sample_interval, "include_events": subscription.include_events}
                    for subscription in self.output_subscriptions
                ],
            },
        )
    ####

    def replay(self, frames: Sequence[ReplayFrame]) -> tuple[InteractiveSnapshot, ...]:
        """Replay a command stream from the current session boundary."""

        if self.status is InteractiveStatus.CREATED:
            pass
        elif self.status is InteractiveStatus.PAUSED:
            self.resume()
        else:
            raise RuntimeError("replay requires a newly created or paused session")
        return tuple(self.step(frame.duration, frame.commands) for frame in frames)
    ####
