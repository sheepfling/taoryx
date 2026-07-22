"""Generic segment, controller, plant, and control-allocation contracts.

This module is deliberately independent of vehicle family.  Fixed-wing,
rotorcraft, rocket-plane, point-mass, and kinematic adapters can implement the
same interfaces without adding syntax to the historical problem language.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, cast

from .contracts import Vector3


@dataclass(frozen=True, slots=True)
class VehicleObservation:
    """Canonical controller observation at one integration instant."""

    time_s: float
    state: Mapping[str, float]
    position: Vector3 | None = None
    velocity: Vector3 | None = None
    body_rates: Vector3 | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.time_s):
            raise ValueError("observation time must be finite")
        if any(not math.isfinite(float(value)) for value in self.state.values()):
            raise ValueError("observation state must contain only finite values")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlDemand:
    """Named, pre-allocation controller demand."""

    values: Mapping[str, float]
    source: str = ""
    saturated: bool = False

    def __post_init__(self) -> None:
        if any(not math.isfinite(float(value)) for value in self.values.values()):
            raise ValueError("control demand must contain only finite values")
        ####
    ####


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """Named, achieved actuator command after allocation and limits."""

    values: Mapping[str, float]
    saturated: tuple[str, ...] = ()
    source: str = ""

    def __post_init__(self) -> None:
        if any(not math.isfinite(float(value)) for value in self.values.values()):
            raise ValueError("control command must contain only finite values")
        ####
    ####


@dataclass(frozen=True, slots=True)
class PlantEvaluation:
    """Common plant result consumed by controllers and evidence writers."""

    force_body_n: Vector3 = Vector3(0.0, 0.0, 0.0)
    moment_body_nm: Vector3 = Vector3(0.0, 0.0, 0.0)
    mass_rate_kg_s: float = 0.0
    diagnostics: Mapping[str, float | str | bool] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not math.isfinite(self.mass_rate_kg_s) or self.mass_rate_kg_s < 0.0:
            raise ValueError("plant mass rate must be finite and nonnegative")
        ####
    ####


class VehiclePlant(Protocol):
    """Evaluate one vehicle family through a canonical plant boundary."""

    def evaluate(self, observation: VehicleObservation, command: ControlCommand) -> PlantEvaluation:
        """Return force, moment, mass-flow, and diagnostics for one sample."""
        ...


class Controller(Protocol):
    """Produce a named control demand from an observation and segment target."""

    def reset(self, segment: SegmentPlan) -> None:
        """Reset controller state at a segment boundary."""
        ...

    def update(self, observation: VehicleObservation, target: Mapping[str, float], dt_s: float) -> ControlDemand:
        """Compute a demand for the current segment sample."""
        ...


class ControlAllocator(Protocol):
    """Map family-independent demands into vehicle-specific actuators."""

    def allocate(self, demand: ControlDemand, observation: VehicleObservation) -> ControlCommand:
        """Apply the vehicle's actuator mapping and limits."""
        ...


@dataclass(frozen=True, slots=True)
class SegmentPlan:
    """One time-bounded control segment shared by all vehicle families."""

    identifier: str
    start_time_s: float
    end_time_s: float
    target: Mapping[str, float] = field(default_factory=dict)
    controller_name: str = ""

    def __post_init__(self) -> None:
        if not self.identifier or self.start_time_s < 0.0 or self.end_time_s <= self.start_time_s:
            raise ValueError("segment requires a nonempty identifier and positive time interval")
        if any(not math.isfinite(float(value)) for value in self.target.values()):
            raise ValueError("segment targets must contain only finite values")
        ####
    ####

    def active(self, time_s: float) -> bool:
        """Return whether the segment owns the supplied time."""

        return self.start_time_s <= time_s < self.end_time_s
        ####
    ####


@dataclass(frozen=True, slots=True)
class SegmentTransition:
    """Controller-visible record of a segment activation."""

    previous_identifier: str | None
    current_identifier: str
    time_s: float
    reason: str = "schedule"
    ####
####


@dataclass(frozen=True, slots=True)
class SegmentSchedule:
    """Validated ordered schedule used by the generic segment controller."""

    segments: tuple[SegmentPlan, ...]

    def __post_init__(self) -> None:
        if not self.segments:
            raise ValueError("segment schedule requires at least one segment")
        ordered = tuple(sorted(self.segments, key=lambda segment: segment.start_time_s))
        if ordered != self.segments:
            raise ValueError("segment schedule must be ordered by start time")
        for previous, current in zip(self.segments, self.segments[1:], strict=False):
            if current.start_time_s < previous.end_time_s:
                raise ValueError("segment schedule contains overlapping segments")
        ####
    ####

    def at(self, time_s: float) -> SegmentPlan:
        """Return the active segment or raise when the schedule has no owner."""

        for segment in self.segments:
            if segment.active(time_s):
                return segment
        raise ValueError(f"no segment owns time {time_s}")
        ####
    ####


@dataclass(slots=True)
class SegmentController:
    """Run one controller and optional allocator across a segment schedule."""

    controller: Controller
    schedule: SegmentSchedule
    allocator: ControlAllocator | None = None
    _active_identifier: str | None = field(default=None, init=False)
    transition_history: list[SegmentTransition] = field(default_factory=list, init=False)

    def step(self, observation: VehicleObservation, dt_s: float) -> ControlCommand:
        """Select the active segment, reset on transition, and issue a command."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("controller step must be positive and finite")
        segment = self.schedule.at(observation.time_s)
        if self._active_identifier != segment.identifier:
            previous_identifier = self._active_identifier
            self.controller.reset(segment)
            callback = getattr(self.controller, "on_transition", None)
            if callback is not None:
                callback(
                    SegmentTransition(previous_identifier, segment.identifier, observation.time_s)
                )
            self.transition_history.append(
                SegmentTransition(previous_identifier, segment.identifier, observation.time_s)
            )
            self._active_identifier = segment.identifier
        demand = self.controller.update(observation, segment.target, dt_s)
        if self.allocator is None:
            return ControlCommand(dict(demand.values), ("demand",) if demand.saturated else (), demand.source)
        return self.allocator.allocate(demand, observation)
        ####
    ####


@dataclass(slots=True)
class ProportionalHoldController:
    """Small generic named-state hold controller for adapter bring-up."""

    output_by_state: Mapping[str, str]
    gains: Mapping[str, float]
    limits: Mapping[str, tuple[float, float]] = field(default_factory=dict)
    _reset_count: int = field(default=0, init=False)

    def reset(self, segment: SegmentPlan) -> None:
        """Reset observable controller state at a segment boundary."""

        self._reset_count += 1
        ####
    ####

    def update(self, observation: VehicleObservation, target: Mapping[str, float], dt_s: float) -> ControlDemand:
        """Return bounded proportional corrections for mapped state channels."""

        if not math.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("controller step must be positive and finite")
        values: dict[str, float] = {}
        saturated = False
        for state_name, output_name in self.output_by_state.items():
            if state_name not in observation.state or state_name not in target:
                raise KeyError(f"controller requires state and target {state_name!r}")
            raw = self.gains.get(state_name, 0.0) * (float(target[state_name]) - float(observation.state[state_name]))
            if output_name in self.limits:
                lower, upper = self.limits[output_name]
                if lower > upper or not all(math.isfinite(value) for value in (lower, upper)):
                    raise ValueError(f"invalid limits for control {output_name!r}")
                bounded = min(upper, max(lower, raw))
                saturated = saturated or bounded != raw
                raw = bounded
            values[output_name] = raw
        return ControlDemand(values, source="proportional-hold", saturated=saturated)
        ####
    ####


@dataclass(frozen=True, slots=True)
class DirectControlAllocator:
    """Identity allocator for point-mass or directly named actuators."""

    limits: Mapping[str, tuple[float, float]] = field(default_factory=dict)

    def allocate(self, demand: ControlDemand, observation: VehicleObservation) -> ControlCommand:
        """Copy named demands while applying optional actuator bounds."""

        values: dict[str, float] = {}
        saturated = ["demand"] if demand.saturated else []
        for name, value in demand.values.items():
            if name not in self.limits:
                values[name] = value
                continue
            lower, upper = self.limits[name]
            if lower > upper or not all(math.isfinite(item) for item in (lower, upper)):
                raise ValueError(f"invalid limits for control {name!r}")
            bounded = min(upper, max(lower, value))
            if bounded != value:
                saturated.append(name)
            values[name] = bounded
        return ControlCommand(values, tuple(saturated), demand.source)
        ####
    ####


def schedule_from_rows(rows: Sequence[Mapping[str, object]]) -> SegmentSchedule:
    """Build a schedule from YAML/JSON-like rows without vehicle assumptions."""

    segments: list[SegmentPlan] = []
    for row in rows:
        raw_target = row.get("target", {})
        if not isinstance(raw_target, Mapping):
            raise ValueError("segment target must be a mapping")
        segments.append(
            SegmentPlan(
                identifier=str(row["id"]),
                start_time_s=float(cast(Any, row["start_s"])),
                end_time_s=float(cast(Any, row["end_s"])),
                target={str(key): float(cast(Any, value)) for key, value in raw_target.items()},
                controller_name=str(row.get("controller", "")),
            )
        )
    return SegmentSchedule(tuple(segments))
    ####
