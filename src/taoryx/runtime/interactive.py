"""Deterministic, externally driven runtime stepping.

This layer deliberately reuses :mod:`taoryx.runtime.engine`. It adds lifecycle
and command-stream semantics around the existing typed state and integrator
contracts; it is not a second numerical kernel.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING, cast

from taoryx.control import SegmentController, VehicleObservation

from .common import Derivative, RuntimeProblem, RuntimeState
from .engine import integrate_active_vehicles

if TYPE_CHECKING:
    from taoryx.outputs import RunArtifact

    from .sensor_scenario import SensorScenarioRuntime


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
    sensor_runtime: SensorScenarioRuntime | None = field(default=None, init=False, repr=False)
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
        if self.problem.sensor_bus is not None:
            self.problem.sensor_bus.initialize(self.problem)
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
            previous_states = {vehicle.name: vehicle.state for vehicle in self.problem.active_vehicles()}
            integrate_active_vehicles(self.problem, duration)
            if self.problem.sensor_bus is not None:
                self.problem.sensor_bus.accepted_step(self.problem, previous_states)
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

    def attach_sensor_scenario(self, spec: object) -> SensorScenarioRuntime:
        """Attach a provider-neutral sensor sidecar to accepted interactive steps."""

        from .sensor_scenario import SensorScenarioSpec, attach_sensor_scenario

        selected = spec if isinstance(spec, SensorScenarioSpec) else SensorScenarioSpec.from_file(cast(str | Path, spec))
        self.sensor_runtime = attach_sensor_scenario(self.problem, selected)
        return self.sensor_runtime
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
            sensor_execution={} if self.sensor_runtime is None else self.sensor_runtime.artifact(),
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

    def save_checkpoint(self, path: str | Path, *, model_fingerprint: str | None = None) -> Path:
        """Persist the session boundary and command stream without pickling callbacks.

        The runtime graph is restored into a caller-supplied model on load.  The
        optional ``model_fingerprint`` lets a caller bind the checkpoint to a
        resolved case or immutable model manifest; callbacks and controllers are
        deliberately rebuilt by the caller rather than serialized as Python.
        """

        payload: dict[str, object] = {
            "schema_version": 1,
            "model_fingerprint": model_fingerprint,
            "problem": _interactive_problem_payload(self.problem),
            "session": {
                "status": self.status.value,
                "last_commands": dict(self._last_commands),
                "fired_events": [[vehicle, event] for vehicle, event in sorted(self._fired_events)],
                "command_history": [{"duration": frame.duration, "commands": dict(frame.commands)} for frame in self.command_history],
                "snapshots": [_interactive_snapshot_payload(snapshot) for snapshot in self.snapshots],
                "event_history": [event.as_dict() for event in self.event_history],
                "controls": [_control_payload(control) for control in self.controls],
                "status_specs": [_status_payload(status) for status in self.status_specs],
                "output_subscriptions": [_subscription_payload(subscription) for subscription in self.output_subscriptions],
            },
        }
        payload["integrity"] = _checkpoint_fingerprint(payload)
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        serialized = json.dumps(payload, indent=2, sort_keys=True) + "\n"
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=destination.parent, delete=False) as temporary:
            temporary.write(serialized)
            temporary_path = Path(temporary.name)
        os.replace(temporary_path, destination)
        return destination
    ####

    @classmethod
    def load_checkpoint(
        cls,
        path: str | Path,
        problem: RuntimeProblem,
        *,
        model_fingerprint: str | None = None,
        controls: Sequence[ControlSpec] | None = None,
        control_model: ControlModel | None = None,
        segment_controllers: Mapping[str, SegmentController] | None = None,
        status_specs: Sequence[StatusSpec] | None = None,
        event_specs: Sequence[EventSpec] = (),
        output_subscriptions: Sequence[OutputSubscription] | None = None,
    ) -> InteractiveSession:
        """Restore a session into an explicitly supplied executable model.

        ``problem`` and any controller/event callbacks are the executable
        factory boundary.  The checkpoint supplies only immutable data, live
        state, controls, histories, and command semantics.  A supplied model
        fingerprint must match the saved value when either side provides one.
        """

        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("schema_version") != 1:
            raise ValueError(f"unsupported TAORYX interactive checkpoint schema: {payload.get('schema_version')!r}")
        saved_integrity = payload.pop("integrity", None)
        if saved_integrity != _checkpoint_fingerprint(payload):
            raise ValueError("interactive checkpoint integrity verification failed")
        saved_model = payload.get("model_fingerprint")
        if saved_model is not None and model_fingerprint is not None and saved_model != model_fingerprint:
            raise ValueError("interactive checkpoint model fingerprint mismatch")

        _restore_interactive_problem(problem, cast(Mapping[str, object], payload["problem"]))
        session_payload = cast(Mapping[str, object], payload["session"])
        saved_controls = tuple(_control_from_payload(cast(Mapping[str, object], item)) for item in cast(Sequence[object], session_payload.get("controls", ())))
        saved_statuses = tuple(_status_from_payload(cast(Mapping[str, object], item)) for item in cast(Sequence[object], session_payload.get("status_specs", ())))
        saved_subscriptions = tuple(_subscription_from_payload(cast(Mapping[str, object], item)) for item in cast(Sequence[object], session_payload.get("output_subscriptions", ())))
        session = cls(
            problem,
            tuple(controls) if controls is not None else saved_controls,
            control_model=control_model,
            segment_controllers=segment_controllers or {},
            status_specs=tuple(status_specs) if status_specs is not None else saved_statuses,
            event_specs=tuple(event_specs),
            output_subscriptions=tuple(output_subscriptions) if output_subscriptions is not None else saved_subscriptions,
        )
        session.status = InteractiveStatus(str(session_payload.get("status", InteractiveStatus.PAUSED.value)))
        session._last_commands = {str(name): _as_float(value) for name, value in cast(Mapping[str, object], session_payload.get("last_commands", {})).items()}
        session._fired_events = {
            (str(item[0]), str(item[1]))
            for item in cast(Sequence[object], session_payload.get("fired_events", ()))
            if isinstance(item, Sequence) and len(item) == 2
        }
        session.command_history = [
            ReplayFrame(_as_float(cast(Mapping[str, object], item)["duration"]), {str(name): _as_float(value) for name, value in cast(Mapping[str, object], cast(Mapping[str, object], item)["commands"]).items()})
            for item in cast(Sequence[object], session_payload.get("command_history", ()))
        ]
        session.snapshots = [_interactive_snapshot_from_payload(cast(Mapping[str, object], item), problem) for item in cast(Sequence[object], session_payload.get("snapshots", ()))]
        session.event_history = [_runtime_event_from_payload(cast(Mapping[str, object], item)) for item in cast(Sequence[object], session_payload.get("event_history", ()))]
        return session
    ####


def _interactive_problem_payload(problem: RuntimeProblem) -> dict[str, object]:
    """Serialize mutable runtime state while leaving executable callbacks out."""

    from .program import _kinematic_state_payload, _state_payload

    return {
        "print_times": list(problem.print_times),
        "table_knots": list(problem.table_knots),
        "required_truth_times": list(problem.required_truth_times),
        "sensor_clocks": [clock.to_metadata() for clock in problem.sensor_clocks],
        "final_time": problem.final_time,
        "metadata": _json_safe(problem.metadata),
        "event_history": _json_safe(problem.event_history),
        "transition_history": [item.to_metadata() for item in problem.transition_history],
        "sensor_bus": None if problem.sensor_bus is None else problem.sensor_bus.to_metadata(),
        "sensor_rebind": {
            "required": problem.sensor_bus is not None,
            "reason": "external sensor providers and estimator subscribers are intentionally not serialized",
        },
        "vehicles": {
            name: {
                "state": _state_payload(vehicle.state),
                "history": [_state_payload(state) for state in vehicle.history],
                "active": vehicle.active,
                "activation_pending": vehicle.activation_pending,
                "segment_number": vehicle.segment_number,
                "fired_events": sorted(vehicle.fired_events),
                "control_values": _json_safe(vehicle.control_values),
                "parameters": _json_safe(vehicle.parameters),
                "step_size": vehicle.step_size,
                "integrator": vehicle.integrator,
                "absolute_tolerance": vehicle.absolute_tolerance,
                "relative_tolerance": vehicle.relative_tolerance,
                "max_step_size": vehicle.max_step_size,
                "publish_derived_rates": vehicle.publish_derived_rates,
                "kinematic_state": _kinematic_state_payload(vehicle.kinematic_state),
                "dynamics_mode": vehicle.dynamics_mode.value,
            }
            for name, vehicle in problem.vehicles.items()
        },
    }
    ####


def _restore_interactive_problem(problem: RuntimeProblem, payload: Mapping[str, object]) -> None:
    """Restore a checkpoint into a caller-provided executable runtime graph."""

    from .program import _kinematic_state_from_payload, _state_from_payload, _transition_pair_from_payload

    saved_vehicles = cast(Mapping[str, object], payload.get("vehicles", {}))
    if set(saved_vehicles) != set(problem.vehicles):
        raise ValueError("interactive checkpoint vehicle graph does not match the supplied problem")
    problem.print_times = tuple(_as_float(value) for value in cast(Sequence[object], payload.get("print_times", ())))
    problem.table_knots = tuple(_as_float(value) for value in cast(Sequence[object], payload.get("table_knots", ())))
    problem.required_truth_times = tuple(_as_float(value) for value in cast(Sequence[object], payload.get("required_truth_times", ())))
    problem.final_time = float(cast(float | int | str, payload["final_time"])) if payload.get("final_time") is not None else None
    problem.metadata = cast(dict[str, object], _json_safe(payload.get("metadata", {})))
    problem.event_history = [cast(dict[str, object], item) for item in cast(Sequence[object], payload.get("event_history", ())) if isinstance(item, Mapping)]
    problem.transition_history = [
        _transition_pair_from_payload(cast(Mapping[str, object], item))
        for item in cast(Sequence[object], payload.get("transition_history", ()))
    ]
    if payload.get("sensor_bus") is not None:
        problem.metadata["checkpoint_sensor_rebind"] = payload.get(
            "sensor_rebind",
            {
                "required": True,
                "reason": "external sensor providers and estimator subscribers are intentionally not serialized",
            },
        )
    for name, raw in saved_vehicles.items():
        vehicle_payload = cast(Mapping[str, object], raw)
        vehicle = problem.vehicles[name]
        vehicle.state = _state_from_payload(cast(Mapping[str, object], vehicle_payload["state"]), vehicle.state.frame)
        vehicle.history = [
            _state_from_payload(cast(Mapping[str, object], item), vehicle.state.frame)
            for item in cast(Sequence[object], vehicle_payload.get("history", ()))
        ]
        vehicle.active = bool(vehicle_payload["active"])
        vehicle.activation_pending = bool(vehicle_payload["activation_pending"])
        vehicle.segment_number = int(cast(int | str, vehicle_payload["segment_number"]))
        vehicle.fired_events = {str(value) for value in cast(Sequence[object], vehicle_payload.get("fired_events", ())) }
        vehicle.control_values = {str(key): _as_float(value) for key, value in cast(Mapping[str, object], vehicle_payload.get("control_values", {})).items()}
        vehicle.parameters = {str(key): _as_float(value) for key, value in cast(Mapping[str, object], vehicle_payload.get("parameters", {})).items()}
        vehicle.step_size = float(cast(float | int | str, vehicle_payload["step_size"]))
        vehicle.integrator = str(vehicle_payload["integrator"])
        vehicle.absolute_tolerance = float(cast(float | int | str, vehicle_payload["absolute_tolerance"]))
        vehicle.relative_tolerance = float(cast(float | int | str, vehicle_payload["relative_tolerance"]))
        vehicle.max_step_size = float(cast(float | int | str, vehicle_payload["max_step_size"])) if vehicle_payload.get("max_step_size") is not None else None
        vehicle.publish_derived_rates = bool(vehicle_payload["publish_derived_rates"])
        sidecar = vehicle_payload.get("kinematic_state")
        if sidecar is not None:
            if vehicle.kinematic_state is None:
                raise ValueError(f"interactive checkpoint contains a kinematic sidecar for non-kinematic vehicle {name!r}")
            vehicle.kinematic_state = _kinematic_state_from_payload(cast(Mapping[str, object], sidecar))
        elif vehicle.kinematic_state is not None:
            raise ValueError(f"interactive checkpoint is missing the kinematic sidecar for vehicle {name!r}")
    ####


def _interactive_snapshot_payload(snapshot: InteractiveSnapshot) -> dict[str, object]:
    """Serialize one interactive snapshot with complete state names and frames."""

    from .program import _state_payload

    return {
        "time_start": snapshot.time_start,
        "time_end": snapshot.time_end,
        "states": {name: _state_payload(state) for name, state in snapshot.states.items()},
        "commands": [
            {"name": command.name, "requested": command.requested, "applied": command.applied, "unit": command.unit, "clamped": command.clamped}
            for command in snapshot.commands
        ],
        "events": list(snapshot.events),
        "diagnostics": list(snapshot.diagnostics),
        "status": snapshot.status.value,
        "runtime_events": [event.as_dict() for event in snapshot.runtime_events],
        "statuses": {vehicle: dict(values) for vehicle, values in snapshot.statuses.items()},
    }
    ####


def _interactive_snapshot_from_payload(payload: Mapping[str, object], problem: RuntimeProblem) -> InteractiveSnapshot:
    """Restore one interactive snapshot against the supplied runtime graph."""

    from .program import _state_from_payload

    states = {
        name: _state_from_payload(cast(Mapping[str, object], raw), problem.vehicles[name].state.frame)
        for name, raw in cast(Mapping[str, object], payload.get("states", {})).items()
    }
    commands = tuple(
        AppliedCommand(
            str(item["name"]),
            float(cast(float | int | str, item["requested"])),
            float(cast(float | int | str, item["applied"])),
            str(item["unit"]) if item.get("unit") is not None else None,
            bool(item.get("clamped", False)),
        )
        for item in (cast(Mapping[str, object], value) for value in cast(Sequence[object], payload.get("commands", ())))
    )
    return InteractiveSnapshot(
        float(cast(float | int | str, payload["time_start"])),
        float(cast(float | int | str, payload["time_end"])),
        states,
        commands,
        tuple(str(value) for value in cast(Sequence[object], payload.get("events", ()))),
        tuple(str(value) for value in cast(Sequence[object], payload.get("diagnostics", ()))),
        InteractiveStatus(str(payload.get("status", InteractiveStatus.RUNNING.value))),
        tuple(_runtime_event_from_payload(cast(Mapping[str, object], value)) for value in cast(Sequence[object], payload.get("runtime_events", ()))),
        {str(vehicle): {str(key): _as_float(value) for key, value in cast(Mapping[str, object], values).items()} for vehicle, values in cast(Mapping[str, object], payload.get("statuses", {})).items()},
    )
    ####


def _control_payload(control: ControlSpec) -> dict[str, object]:
    return {"name": control.name, "unit": control.unit, "default": control.default, "lower": control.lower, "upper": control.upper, "slew_rate": control.slew_rate, "modes": list(control.modes)}
    ####


def _control_from_payload(payload: Mapping[str, object]) -> ControlSpec:
    return ControlSpec(
        str(payload["name"]),
        str(payload["unit"]) if payload.get("unit") is not None else None,
        float(cast(float | int | str, payload["default"])),
        float(cast(float | int | str, payload["lower"])),
        float(cast(float | int | str, payload["upper"])),
        float(cast(float | int | str, payload["slew_rate"])) if payload.get("slew_rate") is not None else None,
        tuple(str(value) for value in cast(Sequence[object], payload.get("modes", ()))),
    )
    ####


def _status_payload(status: StatusSpec) -> dict[str, object]:
    return {"name": status.name, "source": status.source, "unit": status.unit, "modes": list(status.modes)}
    ####


def _status_from_payload(payload: Mapping[str, object]) -> StatusSpec:
    return StatusSpec(str(payload["name"]), str(payload["source"]) if payload.get("source") is not None else None, str(payload["unit"]) if payload.get("unit") is not None else None, tuple(str(value) for value in cast(Sequence[object], payload.get("modes", ()))) )
    ####


def _subscription_payload(subscription: OutputSubscription) -> dict[str, object]:
    return {"channels": list(subscription.channels), "sample_interval": subscription.sample_interval, "include_events": subscription.include_events}
    ####


def _subscription_from_payload(payload: Mapping[str, object]) -> OutputSubscription:
    return OutputSubscription(tuple(str(value) for value in cast(Sequence[object], payload.get("channels", ()))), float(cast(float | int | str, payload["sample_interval"])) if payload.get("sample_interval") is not None else None, bool(payload.get("include_events", True)))
    ####


def _runtime_event_from_payload(payload: Mapping[str, object]) -> RuntimeEvent:
    return RuntimeEvent(
        str(payload["name"]),
        str(payload["vehicle"]),
        float(cast(float | int | str, payload["time"])),
        EventAction(str(payload["action"])),
        str(payload["signal"]) if payload.get("signal") is not None else None,
        str(payload["source"]) if payload.get("source") is not None else None,
        int(cast(int | str, payload["segment_from"])) if payload.get("segment_from") is not None else None,
        int(cast(int | str, payload["segment_to"])) if payload.get("segment_to") is not None else None,
    )
    ####


def _json_safe(value: object) -> object:
    """Keep checkpoint metadata JSON-compatible without serializing callbacks."""

    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [_json_safe(item) for item in value]
    return str(value)
    ####


def _checkpoint_fingerprint(payload: Mapping[str, object]) -> str:
    """Hash a checkpoint payload excluding its integrity field."""

    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()
    ####


def _as_float(value: object) -> float:
    """Convert a JSON scalar to a finite runtime float."""

    return float(cast(float | int | str, value))
    ####
