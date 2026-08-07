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
from dataclasses import dataclass, field, replace
from enum import StrEnum
from pathlib import Path
from typing import TYPE_CHECKING, cast

from taoryx.control import SegmentController, VehicleObservation

from .common import ControlEvaluationRecord, ControlIntervalRecord, Derivative, EventCondition, RuntimeProblem, RuntimeState
from .engine import get_next_step_boundary, integrate_active_vehicles
from .events import refine_segment_final_condition

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
    accepted_start: float | None = None
    accepted_end: float | None = None

    @property
    def accepted_duration(self) -> float | None:
        """Return the truth interval over which this command was accepted."""

        if self.accepted_start is None or self.accepted_end is None:
            return None
        return self.accepted_end - self.accepted_start
        ####

    def as_dict(self) -> dict[str, object]:
        """Return a stable requested/applied/accepted command record."""

        return {
            "name": self.name,
            "requested": self.requested,
            "applied": self.applied,
            "unit": self.unit,
            "clamped": self.clamped,
            "accepted_start": self.accepted_start,
            "accepted_end": self.accepted_end,
            "accepted_duration": self.accepted_duration,
        }
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
    residual: Callable[[RuntimeState], float] | None = None

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
class AcceptedBoundaryRecord:
    """One accepted internal truth interval inside an external step request."""

    time_start: float
    time_end: float
    integration_cadence: float
    integrator: str
    reasons: tuple[str, ...]
    print_cadence: float | None = None

    @property
    def accepted_duration(self) -> float:
        return self.time_end - self.time_start
        ####

    def as_dict(self) -> dict[str, object]:
        return {
            "time_start": self.time_start,
            "time_end": self.time_end,
            "accepted_duration": self.accepted_duration,
            "integration_cadence": self.integration_cadence,
            "integrator": self.integrator,
            "reasons": list(self.reasons),
            "print_cadence": self.print_cadence,
        }
    ####
####


@dataclass(frozen=True, slots=True)
class ReplayFrame:
    duration: float
    commands: Mapping[str, float]

    def as_dict(self) -> dict[str, object]:
        """Return the exact requested command frame used for replay identity."""

        return {"duration": self.duration, "commands": dict(sorted(self.commands.items()))}
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
    requested_duration: float = 0.0
    accepted_duration: float = 0.0
    boundary_reason: str = "requested_external_duration"
    boundary_reasons: tuple[str, ...] = ()
    event_truncated: bool = False
    accepted_boundaries: tuple[AcceptedBoundaryRecord, ...] = ()
    integrator: str | None = None
    integration_cadence: float | None = None
    print_cadence: float | None = None
    replay_identity: str | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "time_start": self.time_start,
            "time_end": self.time_end,
            "status": self.status.value,
            "commands": [command.as_dict() for command in self.commands],
            "requested_duration": self.requested_duration,
            "accepted_duration": self.accepted_duration,
            "boundary_reason": self.boundary_reason,
            "boundary_reasons": list(self.boundary_reasons),
            "event_truncated": self.event_truncated,
            "accepted_boundaries": [boundary.as_dict() for boundary in self.accepted_boundaries],
            "integrator": self.integrator,
            "integration_cadence": self.integration_cadence,
            "print_cadence": self.print_cadence,
            "replay_identity": self.replay_identity,
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
    model_fingerprint: str | None = None
    command_stream_sha256: str | None = None
    replay_identity: str | None = None

    def write_json(self, path: str | Path) -> Path:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": self.schema_version,
            "status": self.status.value,
            "model_fingerprint": self.model_fingerprint,
            "command_stream_sha256": self.command_stream_sha256,
            "replay_identity": self.replay_identity,
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
    model_fingerprint: str | None = None
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
        self.model_fingerprint = self.model_fingerprint or _runtime_model_fingerprint(
            self.problem,
            controls=self.controls,
            event_specs=self.event_specs,
            control_model=self.control_model,
            segment_controller_names=tuple(sorted(self.segment_controllers)),
        )
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

    @property
    def command_stream_sha256(self) -> str:
        """Hash the exact requested external command stream in order."""

        return _identity_digest([frame.as_dict() for frame in self.command_history])
        ####

    @property
    def replay_identity(self) -> str:
        """Return the model- and command-stream-bound replay identity."""

        return _identity_digest(
            {
                "schema": "taoryx.interactive-replay/v1alpha1",
                "model_fingerprint": self.model_fingerprint,
                "command_stream_sha256": self.command_stream_sha256,
            }
        )
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
        """Advance a requested duration using a boundary-applied command set.

        The returned snapshot distinguishes the external duration request from
        the accepted truth interval.  Explicit event residuals are refined to
        an accepted boundary before the event is applied; predicate-only event
        specs retain their historical end-of-boundary behavior.
        """

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
        for vehicle in self.problem.vehicles.values():
            vehicle.control_values = {**vehicle.control_values, **self._last_commands}
            vehicle.control_values_time_s = start
        self.status = InteractiveStatus.RUNNING
        events: list[str] = []
        runtime_events: list[RuntimeEvent] = []
        accepted_boundaries: list[AcceptedBoundaryRecord] = []
        stop_event_seen = False
        try:
            requested_end = start + duration
            final_time = self.problem.final_time
            target_end = requested_end if final_time is None else min(requested_end, final_time)
            while self.time < target_end - 1.0e-12:
                active = self.problem.active_vehicles()
                if not active:
                    self.status = InteractiveStatus.COMPLETED
                    break
                candidate_step = min(vehicle.step_size for vehicle in active)
                remaining = target_end - self.time
                scheduled_boundary = get_next_step_boundary(self.problem, candidate_step, now=self.time)
                scheduled_step = (
                    candidate_step
                    if scheduled_boundary.reason == "integration_cadence"
                    else scheduled_boundary.time - self.time
                )
                accepted_step = min(scheduled_step, remaining)
                if accepted_step <= 1.0e-15:
                    raise RuntimeError("interactive session stalled before its requested accepted-truth boundary")
                previous_states = {vehicle.name: vehicle.state for vehicle in active}
                previous_step_sizes = {vehicle.name: vehicle.step_size for vehicle in active}
                before_step = self.time
                integrate_active_vehicles(self.problem, accepted_step)
                if self.time <= before_step + 1.0e-15:
                    raise RuntimeError("interactive session integration produced no accepted time advance")

                residual_crossings: list[tuple[str, EventSpec, float]] = []
                for vehicle in active:
                    for spec in self.event_specs:
                        if spec.residual is None:
                            continue
                        key = (vehicle.name, spec.name)
                        if spec.once and key in self._fired_events:
                            continue
                        condition = EventCondition(spec.name, spec.residual, action=spec.action.value, source=spec.source)
                        crossing = refine_segment_final_condition(previous_states[vehicle.name], vehicle.state, (condition,))
                        residual_crossings.extend((vehicle.name, spec, item.time) for item in crossing)

                first_crossing_time = min((item[2] for item in residual_crossings), default=None)
                event_boundary = first_crossing_time is not None and first_crossing_time < self.time - 1.0e-10
                if event_boundary:
                    assert first_crossing_time is not None
                    for vehicle in active:
                        vehicle.state = previous_states[vehicle.name]
                        if vehicle.history and vehicle.history[-1].time > before_step + 1.0e-12:
                            vehicle.history.pop()
                        vehicle.discard_control_provenance_after(before_step)
                        vehicle.step_size = previous_step_sizes[vehicle.name]
                    restart_step = first_crossing_time - before_step
                    if restart_step > 1.0e-15:
                        integrate_active_vehicles(self.problem, restart_step)
                    accepted_step = first_crossing_time - before_step
                    boundary_reasons = ["event_boundary"]
                else:
                    boundary_reasons = []
                    if scheduled_boundary.time <= target_end + 1.0e-12:
                        boundary_reasons.append(scheduled_boundary.reason)
                    if first_crossing_time is not None and abs(first_crossing_time - self.time) <= 1.0e-9:
                        boundary_reasons.append("event_boundary")
                    if target_end <= scheduled_boundary.time + 1.0e-12:
                        boundary_reasons.append("requested_external_duration")
                    if not boundary_reasons:
                        boundary_reasons.append("requested_external_duration")
                if self.problem.sensor_bus is not None:
                    self.problem.sensor_bus.accepted_step(self.problem, previous_states)
                accepted_boundaries.append(
                    AcceptedBoundaryRecord(
                        before_step,
                        self.time,
                        candidate_step,
                        _accepted_integrator(active),
                        tuple(dict.fromkeys(boundary_reasons)),
                        _print_cadence(active),
                    )
                )
                for vehicle in self.problem.active_vehicles():
                    for event in vehicle.events:
                        if event.predicate is not None and event.predicate(vehicle.state):
                            events.append(f"{vehicle.name}:{event.name}")
                            if event.action == "stop":
                                vehicle.active = False
                                stop_event_seen = True
                for vehicle in self.problem.vehicles.values():
                    for spec in self.event_specs:
                        key = (vehicle.name, spec.name)
                        if spec.once and key in self._fired_events:
                            continue
                        crossing_at_boundary = any(
                            candidate_vehicle == vehicle.name
                            and candidate_spec.name == spec.name
                            and abs(candidate_time - vehicle.state.time) <= 1.0e-9
                            for candidate_vehicle, candidate_spec, candidate_time in residual_crossings
                        )
                        if not crossing_at_boundary and not spec.predicate(vehicle.state):
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
                            stop_event_seen = True
                if stop_event_seen:
                    self.status = InteractiveStatus.COMPLETED
                    break
            if final_time is not None and self.time >= final_time - 1.0e-12:
                self.status = InteractiveStatus.COMPLETED
        except Exception:
            self.status = InteractiveStatus.FAILED
            raise
        # Replay the requested stream, not only the bounded result. The
        # limiter must be re-applied so the replay verifies control semantics.
        self.command_history.append(ReplayFrame(duration, dict(requested_commands)))
        accepted_commands = tuple(
            replace(command, accepted_start=start, accepted_end=self.time)
            for command in applied
        )
        aggregate_boundary_reasons = tuple(
            dict.fromkeys(reason for boundary in accepted_boundaries for reason in boundary.reasons)
        )
        event_truncated = self.time < target_end - 1.0e-10 and "event_boundary" in aggregate_boundary_reasons
        if event_truncated:
            boundary_reason = "event_boundary"
        elif final_time is not None and target_end < requested_end - 1.0e-12 and self.time >= target_end - 1.0e-12:
            boundary_reason = "final_time"
        else:
            boundary_reason = "requested_external_duration"
        integrators = tuple(dict.fromkeys(boundary.integrator for boundary in accepted_boundaries))
        integration_cadences = tuple(boundary.integration_cadence for boundary in accepted_boundaries)
        print_cadences = tuple(boundary.print_cadence for boundary in accepted_boundaries if boundary.print_cadence is not None)
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
            accepted_commands,
            tuple(events) + tuple(event.name for event in runtime_events),
            diagnostics=tuple(controller_diagnostics),
            status=self.status,
            runtime_events=tuple(runtime_events),
            statuses=statuses,
            requested_duration=duration,
            accepted_duration=self.time - start,
            boundary_reason=boundary_reason,
            boundary_reasons=aggregate_boundary_reasons,
            event_truncated=event_truncated,
            accepted_boundaries=tuple(accepted_boundaries),
            integrator=integrators[0] if len(integrators) == 1 else ("mixed" if integrators else None),
            integration_cadence=min(integration_cadences, default=None),
            print_cadence=min(print_cadences, default=None),
            replay_identity=self.replay_identity,
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
        return InteractiveArtifact(
            2,
            self.status,
            tuple(self.snapshots),
            self.model_fingerprint,
            self.command_stream_sha256,
            self.replay_identity,
        )
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
                model_id=vehicle.model_id,
                parent_model_id=vehicle.parent_model_id,
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
                "model_fingerprint": self.model_fingerprint,
                "command_stream_sha256": self.command_stream_sha256,
                "replay_identity": self.replay_identity,
                "step_records": [
                    {
                        "time_start": snapshot.time_start,
                        "time_end": snapshot.time_end,
                        "requested_duration": snapshot.requested_duration,
                        "accepted_duration": snapshot.accepted_duration,
                        "boundary_reason": snapshot.boundary_reason,
                        "boundary_reasons": list(snapshot.boundary_reasons),
                        "event_truncated": snapshot.event_truncated,
                        "commands": [command.as_dict() for command in snapshot.commands],
                        "accepted_boundaries": [boundary.as_dict() for boundary in snapshot.accepted_boundaries],
                        "replay_identity": snapshot.replay_identity,
                    }
                    for snapshot in self.snapshots
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

        if model_fingerprint is not None:
            self.model_fingerprint = model_fingerprint
        payload: dict[str, object] = {
            "schema_version": 2,
            "model_fingerprint": self.model_fingerprint,
            "problem": _interactive_problem_payload(self.problem),
            "session": {
                "status": self.status.value,
                "last_commands": dict(self._last_commands),
                "fired_events": [[vehicle, event] for vehicle, event in sorted(self._fired_events)],
                "command_history": [frame.as_dict() for frame in self.command_history],
                "command_stream_sha256": self.command_stream_sha256,
                "replay_identity": self.replay_identity,
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
        if payload.get("schema_version") not in {1, 2}:
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
            model_fingerprint=str(saved_model) if saved_model is not None else model_fingerprint,
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
        saved_stream = session_payload.get("command_stream_sha256")
        if saved_stream is not None and str(saved_stream) != session.command_stream_sha256:
            raise ValueError("interactive checkpoint command stream identity mismatch")
        saved_replay = session_payload.get("replay_identity")
        if saved_replay is not None and str(saved_replay) != session.replay_identity:
            raise ValueError("interactive checkpoint replay identity mismatch")
        return session
    ####


def _interactive_problem_payload(problem: RuntimeProblem) -> dict[str, object]:
    """Serialize mutable runtime state while leaving executable callbacks out."""

    from .program import _kinematic_state_payload, _state_payload
    from .sensor_scenario import sensor_scenario_checkpoint_payload

    sensor_scenario_checkpoint = sensor_scenario_checkpoint_payload(problem)

    return {
        "print_times": list(problem.print_times),
        "table_knots": list(problem.table_knots),
        "required_truth_times": list(problem.required_truth_times),
        "sensor_clocks": [clock.to_metadata() for clock in problem.sensor_clocks],
        "final_time": problem.final_time,
        "metadata": _json_safe(
            {
                key: value
                for key, value in problem.metadata.items()
                if key != "_sensor_scenario_runtime"
            }
        ),
        "event_history": _json_safe(problem.event_history),
        "transition_history": [item.to_metadata() for item in problem.transition_history],
        "sensor_bus": None if problem.sensor_bus is None else problem.sensor_bus.to_metadata(),
        "sensor_scenario_checkpoint": sensor_scenario_checkpoint,
        "sensor_rebind": {
            "required": problem.sensor_bus is not None and sensor_scenario_checkpoint is None,
            "reason": (
                "sensor bus is not a registered declared-scenario checkpoint"
                if sensor_scenario_checkpoint is None
                else "declared sensor scenario is restored from checkpoint"
            ),
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
                "control_values_time_s": vehicle.control_values_time_s,
                "control_evaluation_history": [record.as_dict() for record in vehicle.control_evaluation_history],
                "control_interval_history": [record.as_dict() for record in vehicle.control_interval_history],
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
    if payload.get("sensor_bus") is not None and payload.get("sensor_scenario_checkpoint") is None:
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
        vehicle.control_values_time_s = _as_float(vehicle_payload.get("control_values_time_s", vehicle.state.time))
        vehicle.control_evaluation_history = [
            ControlEvaluationRecord.from_dict(item)
            for item in cast(Sequence[object], vehicle_payload.get("control_evaluation_history", ()))
            if isinstance(item, Mapping)
        ]
        vehicle.control_interval_history = [
            ControlIntervalRecord.from_dict(item)
            for item in cast(Sequence[object], vehicle_payload.get("control_interval_history", ()))
            if isinstance(item, Mapping)
        ]
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
    raw_sensor_checkpoint = payload.get("sensor_scenario_checkpoint")
    if raw_sensor_checkpoint is not None:
        if not isinstance(raw_sensor_checkpoint, Mapping):
            raise ValueError("interactive checkpoint sensor_scenario_checkpoint must be a mapping")
        from .sensor_scenario import restore_sensor_scenario_checkpoint

        restore_sensor_scenario_checkpoint(problem, raw_sensor_checkpoint)
    ####


def _interactive_snapshot_payload(snapshot: InteractiveSnapshot) -> dict[str, object]:
    """Serialize one interactive snapshot with complete state names and frames."""

    from .program import _state_payload

    return {
        "time_start": snapshot.time_start,
        "time_end": snapshot.time_end,
        "states": {name: _state_payload(state) for name, state in snapshot.states.items()},
        "commands": [command.as_dict() for command in snapshot.commands],
        "events": list(snapshot.events),
        "diagnostics": list(snapshot.diagnostics),
        "status": snapshot.status.value,
        "runtime_events": [event.as_dict() for event in snapshot.runtime_events],
        "statuses": {vehicle: dict(values) for vehicle, values in snapshot.statuses.items()},
        "requested_duration": snapshot.requested_duration,
        "accepted_duration": snapshot.accepted_duration,
        "boundary_reason": snapshot.boundary_reason,
        "boundary_reasons": list(snapshot.boundary_reasons),
        "event_truncated": snapshot.event_truncated,
        "accepted_boundaries": [boundary.as_dict() for boundary in snapshot.accepted_boundaries],
        "integrator": snapshot.integrator,
        "integration_cadence": snapshot.integration_cadence,
        "print_cadence": snapshot.print_cadence,
        "replay_identity": snapshot.replay_identity,
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
            float(cast(float | int | str, item["accepted_start"])) if item.get("accepted_start") is not None else None,
            float(cast(float | int | str, item["accepted_end"])) if item.get("accepted_end") is not None else None,
        )
        for item in (cast(Mapping[str, object], value) for value in cast(Sequence[object], payload.get("commands", ())))
    )
    accepted_boundaries = tuple(
        AcceptedBoundaryRecord(
            float(cast(float | int | str, item["time_start"])),
            float(cast(float | int | str, item["time_end"])),
            float(cast(float | int | str, item["integration_cadence"])),
            str(item["integrator"]),
            tuple(str(value) for value in cast(Sequence[object], item.get("reasons", ()))),
            float(cast(float | int | str, item["print_cadence"])) if item.get("print_cadence") is not None else None,
        )
        for item in (cast(Mapping[str, object], value) for value in cast(Sequence[object], payload.get("accepted_boundaries", ())))
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
        float(cast(float | int | str, payload.get("requested_duration", 0.0))),
        float(cast(float | int | str, payload.get("accepted_duration", 0.0))),
        str(payload.get("boundary_reason", "requested_external_duration")),
        tuple(str(value) for value in cast(Sequence[object], payload.get("boundary_reasons", ()))),
        bool(payload.get("event_truncated", False)),
        accepted_boundaries,
        str(payload["integrator"]) if payload.get("integrator") is not None else None,
        float(cast(float | int | str, payload["integration_cadence"])) if payload.get("integration_cadence") is not None else None,
        float(cast(float | int | str, payload["print_cadence"])) if payload.get("print_cadence") is not None else None,
        str(payload["replay_identity"]) if payload.get("replay_identity") is not None else None,
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


def _accepted_integrator(vehicles: Sequence[object]) -> str:
    """Return the integrator identity for one synchronized accepted interval."""

    names = tuple(dict.fromkeys(str(getattr(vehicle, "integrator")) for vehicle in vehicles))
    return names[0] if len(names) == 1 else ("mixed" if names else "unknown")
    ####


def _print_cadence(vehicles: Sequence[object]) -> float | None:
    """Read the source-declared output cadence without treating it as ``dt``."""

    cadences = [
        float(value)
        for vehicle in vehicles
        for value in (getattr(vehicle, "state").named.get("_dtprnt"),)
        if isinstance(value, (int, float)) and math.isfinite(float(value)) and float(value) > 0.0
    ]
    return min(cadences, default=None)
    ####


def _callable_identity(callback: object) -> object:
    """Describe executable callbacks without including process-local addresses."""

    if callback is None:
        return None
    module = getattr(callback, "__module__", type(callback).__module__)
    name = getattr(callback, "__qualname__", type(callback).__qualname__)
    code = getattr(callback, "__code__", None)
    return {
        "module": str(module),
        "qualname": str(name),
        "bytecode": code.co_code.hex() if code is not None else None,
        "constants": _stable_identity_value(code.co_consts) if code is not None else None,
    }
    ####


def _stable_identity_value(value: object) -> object:
    """Convert model identity inputs into deterministic JSON-compatible data."""

    if isinstance(value, float) and not math.isfinite(value):
        return f"nonfinite:{value!r}"
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(key): _stable_identity_value(item) for key, item in sorted(value.items(), key=lambda item: str(item[0]))}
    if isinstance(value, (set, frozenset)):
        return [_stable_identity_value(item) for item in sorted(value, key=repr)]
    if isinstance(value, (list, tuple)):
        return [_stable_identity_value(item) for item in value]
    if callable(value):
        return _callable_identity(value)
    enum_value = getattr(value, "value", None)
    if enum_value is not None and isinstance(enum_value, (str, int, float, bool)):
        return enum_value
    return {"type": f"{type(value).__module__}.{type(value).__qualname__}"}
    ####


def _runtime_model_fingerprint(
    problem: RuntimeProblem,
    *,
    controls: Sequence[ControlSpec] = (),
    event_specs: Sequence[EventSpec] = (),
    control_model: ControlModel | None = None,
    segment_controller_names: Sequence[str] = (),
) -> str:
    """Derive a stable default fingerprint from the executable model contract."""

    metadata = {
        key: value
        for key, value in problem.metadata.items()
        if key not in {"tables", "_sensor_scenario_runtime"}
    }
    payload = {
        "schema": "taoryx.interactive-model/v1alpha1",
        "problem": {
            "print_times": list(problem.print_times),
            "table_knots": list(problem.table_knots),
            "required_truth_times": list(problem.required_truth_times),
            "sensor_clocks": [clock.to_metadata() for clock in problem.sensor_clocks],
            "final_time": problem.final_time,
            "metadata": _stable_identity_value(metadata),
        },
        "interactive_contract": {
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
                for control in controls
            ],
            "events": [
                {
                    "name": event.name,
                    "action": event.action.value,
                    "signal": event.signal,
                    "once": event.once,
                    "source": event.source,
                    "predicate": _callable_identity(event.predicate),
                    "residual": _callable_identity(event.residual),
                }
                for event in event_specs
            ],
            "control_model": _callable_identity(control_model),
            "segment_controller_names": list(segment_controller_names),
        },
        "vehicles": [
            {
                "name": vehicle.name,
                "model_id": vehicle.model_id,
                "vehicle_kind": str(vehicle.vehicle_kind),
                "dynamics_mode": str(vehicle.dynamics_mode),
                "state": {
                    "time": vehicle.state.time,
                    "values": list(vehicle.state.values),
                    "frame": str(vehicle.state.frame),
                    "named": _stable_identity_value(vehicle.state.named),
                    "value_names": list(vehicle.state.value_names),
                },
                "step_size": vehicle.step_size,
                "integrator": vehicle.integrator,
                "absolute_tolerance": vehicle.absolute_tolerance,
                "relative_tolerance": vehicle.relative_tolerance,
                "max_step_size": vehicle.max_step_size,
                "dependencies": list(vehicle.dependencies),
                "parameters": _stable_identity_value(vehicle.parameters),
                "control_values": _stable_identity_value(vehicle.control_values),
                "events": [
                    {"name": event.name, "action": event.action, "signal": event.signal, "source": event.source}
                    for event in vehicle.events
                ],
                "derivative": _callable_identity(vehicle.derivative),
                "point_mass_derivative": _callable_identity(vehicle.point_mass_derivative),
                "environment_evaluator": _callable_identity(vehicle.environment_evaluator),
            }
            for vehicle in problem.vehicles.values()
        ],
    }
    return _identity_digest(payload)
    ####


def _identity_digest(payload: object) -> str:
    """Hash canonical JSON for replay and model identities."""

    canonical = json.dumps(_stable_identity_value(payload), sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()
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
