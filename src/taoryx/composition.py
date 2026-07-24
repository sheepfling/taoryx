"""Beginner-friendly composition of reusable segments and waypoint courses.

The lower-level taoryx.segmentation models remain the source-compatible
boundary. This module adds a small authoring layer on top: a registry of
reviewed segment templates, a fluent trajectory builder, and structural
evaluation reports that explain what will be compiled before a source problem
is touched.
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.outputs import RunArtifact, SegmentSpan, VehicleTelemetry
from taoryx.segmentation import (
    DynamicsMode,
    GoalKind,
    GoalSpec,
    SegmentationScenario,
    SegmentEntry,
    SegmentExit,
    SegmentSpec,
    TransitionPolicy,
)

CompositionStatus = Literal["pass", "warning", "fail"]
RuntimeStatus = Literal["pass", "warning", "fail", "blocked"]
CheckStatus = Literal["pass", "warning", "fail", "blocked", "not_run"]
SegmentFactory = Callable[..., SegmentSpec]


class WaypointSpec(BaseModel):
    """A simple named waypoint that can be appended to a trajectory."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1, pattern=r"^[A-Za-z][A-Za-z0-9_-]*$")
    target: Mapping[str, float] = Field(min_length=1)
    tolerance: Mapping[str, float] = Field(min_length=1)
    duration_s: float = Field(gt=0.0)
    dwell_time_s: float = Field(default=0.0, ge=0.0)
    controller: str | None = None
    actuator_binding: str | None = None
    reference: str | None = None

    @model_validator(mode="after")
    def validate_values(self) -> WaypointSpec:
        if any(not math.isfinite(float(value)) for value in self.target.values()):
            raise ValueError(f"waypoint {self.id!r} target values must be finite")
        if any(float(value) <= 0.0 or not math.isfinite(float(value)) for value in self.tolerance.values()):
            raise ValueError(f"waypoint {self.id!r} tolerances must be finite and positive")
        if self.dwell_time_s > self.duration_s:
            raise ValueError(f"waypoint {self.id!r} dwell time cannot exceed duration")
        return self
        ####


class SegmentTemplateInfo(BaseModel):
    """Human-readable registry metadata for one composable segment kind."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    goal_kind: GoalKind
    requires_target: bool = False
    requires_tolerance: bool = False
    requires_reference: bool = False
    ####


class SegmentEvaluation(BaseModel):
    """One structural check emitted before a composed scenario is compiled."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    segment_id: str = Field(min_length=1)
    status: CompositionStatus
    checks: tuple[str, ...] = ()
    messages: tuple[str, ...] = ()
    ####


class CompositionReport(BaseModel):
    """Explain whether a composed trajectory is ready for compilation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    scenario_id: str
    status: CompositionStatus
    segments: tuple[SegmentEvaluation, ...]
    checks: tuple[str, ...] = ()
    messages: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """Return whether the composition has no structural failure."""

        return self.status != "fail"

    def raise_for_failure(self) -> None:
        """Raise a concise error when composition is not compile-ready."""

        if self.status == "fail":
            details = "; ".join(self.messages) or "segment composition failed validation"
            raise ValueError(f"composition {self.scenario_id!r} is invalid: {details}")
        ####


class RuntimeEvaluationOptions(BaseModel):
    """Policy knobs for evaluating a recorded composition run.

    Channel names in goals are deliberately not guessed.  Supply aliases when
    a friendly authoring name such as ``altitude_m`` maps to an emitted
    semantic channel such as ``position.altitude.geodetic``.  This keeps the
    evaluator generic across vehicle families and makes missing telemetry
    visible instead of silently scoring the wrong signal.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    channel_aliases: Mapping[str, str] = Field(default_factory=dict)
    required_channels: tuple[str, ...] = ()
    entry_tolerance: float = Field(default=1.0e-6, ge=0.0)
    transition_tolerances: Mapping[str, float] = Field(default_factory=dict)
    max_saturation_fraction: float | None = Field(default=None, ge=0.0, le=1.0)
    require_explicit_segment_spans: bool = False

    @model_validator(mode="after")
    def validate_tolerances(self) -> RuntimeEvaluationOptions:
        if any(not math.isfinite(float(value)) for value in self.transition_tolerances.values()):
            raise ValueError("transition tolerances must be finite")
        if any(not name.strip() for name in self.required_channels):
            raise ValueError("required runtime channel names cannot be empty")
        if len(self.required_channels) != len(set(self.required_channels)):
            raise ValueError("required runtime channels must be unique")
        return self
        ####


class RuntimeCheck(BaseModel):
    """One explainable runtime acceptance check."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    name: str = Field(min_length=1)
    status: CheckStatus
    actual: object | None = None
    expected: object | None = None
    tolerance: float | None = None
    message: str | None = None
    ####


class SegmentRuntimeEvaluation(BaseModel):
    """Evidence for one segment after a trajectory has been recorded."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    segment_id: str
    status: RuntimeStatus
    start_time_s: float | None = None
    end_time_s: float | None = None
    sample_count: int = 0
    entry_checks: tuple[RuntimeCheck, ...] = ()
    goal_checks: tuple[RuntimeCheck, ...] = ()
    transition_checks: tuple[RuntimeCheck, ...] = ()
    exit_checks: tuple[RuntimeCheck, ...] = ()
    quality_checks: tuple[RuntimeCheck, ...] = ()
    messages: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """Return whether this segment has no failed or blocked check."""

        return self.status in {"pass", "warning"}
        ####


class RuntimeCompositionReport(BaseModel):
    """Explain whether recorded telemetry satisfies a composed scenario."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    scenario_id: str
    vehicle_id: str
    status: RuntimeStatus
    segments: tuple[SegmentRuntimeEvaluation, ...]
    checks: tuple[RuntimeCheck, ...] = ()
    messages: tuple[str, ...] = ()

    @property
    def ok(self) -> bool:
        """Return whether no required runtime check failed or was blocked."""

        return self.status in {"pass", "warning"}

    def raise_for_failure(self) -> None:
        """Raise a concise error for failed or unevaluable runtime evidence."""

        if self.status in {"fail", "blocked"}:
            details = "; ".join(self.messages) or "runtime composition evaluation failed"
            raise ValueError(f"runtime composition {self.scenario_id!r} is invalid: {details}")
        ####

def _time_literal(value: float) -> str:
    """Render a finite time without introducing noisy decimal tails."""

    if not math.isfinite(value):
        raise ValueError("segment times must be finite")
    return f"{value:g}"
    ####


def _goal_segment(
    *,
    identifier: str,
    start_time_s: float,
    end_time_s: float,
    mode: DynamicsMode,
    goal_kind: GoalKind,
    target: Mapping[str, float] | None = None,
    tolerance: Mapping[str, float] | None = None,
    dwell_time_s: float = 0.0,
    reference: str | None = None,
    completion_condition: str | None = None,
    controller: str | None = None,
    actuator_binding: str | None = None,
    controls: tuple[str, ...] = (),
    source_segment: int = 1,
    frame: str = "ecfc",
    units: str = "si",
    initial_state: Mapping[str, float] | None = None,
    inherited_from: str | None = None,
    transition: TransitionPolicy = TransitionPolicy(),
    transition_values: Mapping[str, float] | None = None,
) -> SegmentSpec:
    """Build one standard time-windowed goal segment."""

    if end_time_s <= start_time_s:
        raise ValueError("segment duration must be positive")
    goal = GoalSpec(
        id=f"{identifier}-goal",
        kind=goal_kind,
        target={} if target is None else dict(target),
        tolerance={} if tolerance is None else dict(tolerance),
        dwell_time_s=dwell_time_s,
        reference=reference,
        completion_condition=completion_condition,
    )
    return SegmentSpec(
        id=identifier,
        source_segment=source_segment,
        mode=mode,
        entry=SegmentEntry(
            condition=f"time >= {_time_literal(start_time_s)}",
            frame=frame,
            units=units,
        ),
        exit=SegmentExit(condition=f"time > {_time_literal(end_time_s)}"),
        controller=controller,
        actuator_binding=actuator_binding,
        controls=controls,
        initial_state={} if initial_state is None else dict(initial_state),
        inherited_from=inherited_from,
        transition_values={} if transition_values is None else dict(transition_values),
        transition=transition,
        goal=goal,
    )
    ####


def _standard_templates() -> dict[str, tuple[SegmentTemplateInfo, SegmentFactory]]:
    """Return the reviewed built-in segment templates."""

    def trim_hold(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="trim_hold", **cast(Any, kwargs))

    def hover(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="hover", **cast(Any, kwargs))

    def waypoint(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="waypoint", **cast(Any, kwargs))

    def altitude_capture(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="altitude_capture", **cast(Any, kwargs))

    def heading_capture(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="heading_capture", **cast(Any, kwargs))

    def moving_target_intercept(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="intercept_geometry", **cast(Any, kwargs))

    def powered_ascent(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="powered_ascent", **cast(Any, kwargs))

    def ballistic_coast(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="ballistic_coast", **cast(Any, kwargs))

    def bank_maneuver(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="bank_maneuver", **cast(Any, kwargs))

    def alpha_profile(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="alpha_maneuver", **cast(Any, kwargs))

    def skip_maneuver(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="skip_maneuver", **cast(Any, kwargs))

    def terminal_pronav(**kwargs: object) -> SegmentSpec:
        return _goal_segment(goal_kind="terminal_guidance", **cast(Any, kwargs))

    return {
        "trim_hold": (
            SegmentTemplateInfo(
                name="trim_hold",
                description="Hold a declared trimmed state for a bounded dwell.",
                goal_kind="trim_hold",
            ),
            trim_hold,
        ),
        "hover": (
            SegmentTemplateInfo(
                name="hover",
                description="Hold a multirotor hover target for a bounded dwell.",
                goal_kind="hover",
            ),
            hover,
        ),
        "waypoint": (
            SegmentTemplateInfo(
                name="waypoint",
                description="Capture one named target with tolerance and optional dwell.",
                goal_kind="waypoint",
                requires_target=True,
                requires_tolerance=True,
            ),
            waypoint,
        ),
        "altitude_capture": (
            SegmentTemplateInfo(
                name="altitude_capture",
                description="Capture an altitude channel with an explicit tolerance.",
                goal_kind="altitude_capture",
                requires_target=True,
                requires_tolerance=True,
            ),
            altitude_capture,
        ),
        "heading_capture": (
            SegmentTemplateInfo(
                name="heading_capture",
                description="Capture a heading channel with an explicit tolerance.",
                goal_kind="heading_capture",
                requires_target=True,
                requires_tolerance=True,
            ),
            heading_capture,
        ),
        "moving_target_intercept": (
            SegmentTemplateInfo(
                name="moving_target_intercept",
                description="Track a declared moving target using LOS, closure, and intercept-response evidence.",
                goal_kind="intercept_geometry",
                requires_target=True,
                requires_tolerance=True,
                requires_reference=True,
            ),
            moving_target_intercept,
        ),
        "powered_ascent": (
            SegmentTemplateInfo(
                name="powered_ascent",
                description="Run a bounded thrust-driven ascent or boost phase.",
                goal_kind="powered_ascent",
            ),
            powered_ascent,
        ),
        "ballistic_coast": (
            SegmentTemplateInfo(
                name="ballistic_coast",
                description="Propagate a passive or low-control ballistic coast phase.",
                goal_kind="ballistic_coast",
            ),
            ballistic_coast,
        ),
        "bank_maneuver": (
            SegmentTemplateInfo(
                name="bank_maneuver",
                description="Execute a bounded bank or lateral steering maneuver.",
                goal_kind="bank_maneuver",
            ),
            bank_maneuver,
        ),
        "alpha_profile": (
            SegmentTemplateInfo(
                name="alpha_profile",
                description="Execute a bounded angle-of-attack or phugoid profile.",
                goal_kind="alpha_maneuver",
            ),
            alpha_profile,
        ),
        "skip_maneuver": (
            SegmentTemplateInfo(
                name="skip_maneuver",
                description="Execute a bounded skip-flight entry/exit maneuver.",
                goal_kind="skip_maneuver",
            ),
            skip_maneuver,
        ),
        "terminal_pronav": (
            SegmentTemplateInfo(
                name="terminal_pronav",
                description="Hand off to terminal proportional-navigation guidance.",
                goal_kind="terminal_guidance",
                requires_target=True,
                requires_tolerance=True,
                requires_reference=True,
            ),
            terminal_pronav,
        ),
    }
    ####


@dataclass(frozen=True, slots=True)
class SegmentCompositionRegistry:
    """Registry of named segment factories used by the trajectory builder."""

    _templates: Mapping[str, tuple[SegmentTemplateInfo, SegmentFactory]]

    @classmethod
    def standard(cls) -> SegmentCompositionRegistry:
        """Create a registry containing the reviewed built-in templates."""

        return cls(dict(_standard_templates()))
        ####

    def names(self) -> tuple[str, ...]:
        """Return stable template names for menus and agent discovery."""

        return tuple(sorted(self._templates))
        ####

    def describe(self, name: str) -> SegmentTemplateInfo:
        """Return metadata for one template."""

        try:
            return self._templates[name][0]
        except KeyError as error:
            raise KeyError(f"unknown segment template {name!r}; choose from {self.names()}") from error
        ####

    def register(
        self,
        name: str,
        factory: SegmentFactory,
        *,
        description: str,
        goal_kind: GoalKind = "waypoint",
        requires_target: bool = False,
        requires_tolerance: bool = False,
        requires_reference: bool = False,
    ) -> SegmentCompositionRegistry:
        """Return a registry with one additional custom template."""

        if not name or name in self._templates:
            raise ValueError(f"segment template name is empty or already registered: {name!r}")
        templates = dict(self._templates)
        templates[name] = (
            SegmentTemplateInfo(
                name=name,
                description=description,
                goal_kind=goal_kind,
                requires_target=requires_target,
                requires_tolerance=requires_tolerance,
                requires_reference=requires_reference,
            ),
            factory,
        )
        return SegmentCompositionRegistry(templates)
        ####

    def make(self, name: str, **kwargs: object) -> SegmentSpec:
        """Instantiate a registered template with validated factory arguments."""

        try:
            info, factory = self._templates[name]
        except KeyError as error:
            raise KeyError(f"unknown segment template {name!r}; choose from {self.names()}") from error
        if info.requires_target and not kwargs.get("target"):
            raise ValueError(f"segment template {name!r} requires a non-empty target")
        if info.requires_tolerance and not kwargs.get("tolerance"):
            raise ValueError(f"segment template {name!r} requires a non-empty tolerance")
        if info.requires_reference and not kwargs.get("reference"):
            raise ValueError(f"segment template {name!r} requires a moving-target reference")
        return factory(**kwargs)
        ####


class TrajectoryBuilder:
    """Compose a sequential trajectory without hand-writing catalog YAML."""

    def __init__(
        self,
        scenario_id: str,
        *,
        vehicle: str,
        family: str,
        source_problem: str,
        output_problem: str | None = None,
        output_manifest: str | None = None,
        output_audit: str | None = None,
        runtime_tables: Iterable[str] = (),
        mode: DynamicsMode = "rigid_body_6dof",
        registry: SegmentCompositionRegistry | None = None,
    ) -> None:
        self.scenario_id = scenario_id
        self.vehicle = vehicle
        self.family = family
        self.source_problem = source_problem
        self.output_problem = output_problem or f"artifacts/composition/{scenario_id}.prb"
        self.output_manifest = output_manifest or f"artifacts/composition/{scenario_id}.manifest.json"
        self.output_audit = output_audit or f"artifacts/composition/{scenario_id}.audit.json"
        self.runtime_tables = tuple(runtime_tables)
        self.mode = mode
        self.registry = registry or SegmentCompositionRegistry.standard()
        self._segments: list[SegmentSpec] = []
        self._cursor_s = 0.0
        ####

    @property
    def duration_s(self) -> float:
        """Return the end of the current sequential composition."""

        return self._cursor_s
        ####

    def add(self, segment: SegmentSpec) -> TrajectoryBuilder:
        """Append an already-built segment to the sequential composition."""

        if any(existing.id == segment.id for existing in self._segments):
            raise ValueError(f"trajectory {self.scenario_id!r} already contains segment {segment.id!r}")
        if self._segments and segment.entry.condition != f"time >= {_time_literal(self._cursor_s)}":
            raise ValueError(
                "composed segments must use the builder's sequential time boundary; "
                "use add() only for builder-created or explicitly aligned segments"
            )
        self._segments.append(segment)
        self._cursor_s = max(self._cursor_s, _segment_end_time(segment))
        return self
        ####

    def use(
        self,
        template: str,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float] | None = None,
        tolerance: Mapping[str, float] | None = None,
        dwell_time_s: float = 0.0,
        controller: str | None = None,
        actuator_binding: str | None = None,
        reference: str | None = None,
        completion_condition: str | None = None,
        controls: Iterable[str] = (),
        source_segment: int = 1,
        frame: str = "ecfc",
        units: str = "si",
        initial_state: Mapping[str, float] | None = None,
        inherited_from: str | None = None,
        transition: TransitionPolicy = TransitionPolicy(),
        transition_values: Mapping[str, float] | None = None,
    ) -> TrajectoryBuilder:
        """Append one registered template at the next available time."""

        if duration_s <= 0.0 or not math.isfinite(duration_s):
            raise ValueError("segment duration must be finite and positive")
        start = self._cursor_s
        segment = self.registry.make(
            template,
            identifier=identifier,
            start_time_s=start,
            end_time_s=start + duration_s,
            mode=self.mode,
            target=target,
            tolerance=tolerance,
            dwell_time_s=dwell_time_s,
            controller=controller,
            actuator_binding=actuator_binding,
            reference=reference,
            completion_condition=completion_condition,
            controls=tuple(controls),
            source_segment=source_segment,
            frame=frame,
            units=units,
            initial_state=initial_state,
            inherited_from=inherited_from,
            transition=transition,
            transition_values=transition_values,
        )
        self._segments.append(segment)
        self._cursor_s += duration_s
        return self
        ####

    def trim_hold(
        self,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float],
        tolerance: Mapping[str, float],
        controller: str | None = None,
        actuator_binding: str | None = None,
    ) -> TrajectoryBuilder:
        """Append the common trim-hold component without a template name."""

        return self.use(
            "trim_hold",
            identifier,
            duration_s=duration_s,
            target=target,
            tolerance=tolerance,
            controller=controller,
            actuator_binding=actuator_binding,
        )
        ####

    def hover(
        self,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float],
        tolerance: Mapping[str, float],
        controller: str | None = None,
        actuator_binding: str | None = None,
    ) -> TrajectoryBuilder:
        """Append the common hover component without a template name."""

        return self.use(
            "hover",
            identifier,
            duration_s=duration_s,
            target=target,
            tolerance=tolerance,
            controller=controller,
            actuator_binding=actuator_binding,
        )
        ####

    def altitude_capture(
        self,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float],
        tolerance: Mapping[str, float],
        controller: str | None = None,
        actuator_binding: str | None = None,
    ) -> TrajectoryBuilder:
        """Append the common altitude-capture component."""

        return self.use(
            "altitude_capture",
            identifier,
            duration_s=duration_s,
            target=target,
            tolerance=tolerance,
            controller=controller,
            actuator_binding=actuator_binding,
        )
        ####

    def heading_capture(
        self,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float],
        tolerance: Mapping[str, float],
        controller: str | None = None,
        actuator_binding: str | None = None,
    ) -> TrajectoryBuilder:
        """Append the common heading-capture component."""

        return self.use(
            "heading_capture",
            identifier,
            duration_s=duration_s,
            target=target,
            tolerance=tolerance,
            controller=controller,
            actuator_binding=actuator_binding,
        )
        ####

    def moving_target_intercept(
        self,
        identifier: str,
        *,
        duration_s: float,
        target: Mapping[str, float],
        tolerance: Mapping[str, float],
        reference: str,
        controller: str | None = None,
        actuator_binding: str | None = None,
        dwell_time_s: float = 0.0,
    ) -> TrajectoryBuilder:
        """Append a moving-target ProNav/intercept segment.

        The target mapping should normally include a terminal range and
        closing-velocity contract.  The reference names the target state or
        track used by the runtime; it is not inferred from a waypoint ID.
        """

        return self.use(
            "moving_target_intercept",
            identifier,
            duration_s=duration_s,
            target=target,
            tolerance=tolerance,
            reference=reference,
            controller=controller,
            actuator_binding=actuator_binding,
            dwell_time_s=dwell_time_s,
        )
        ####

    def waypoint(self, waypoint: WaypointSpec) -> TrajectoryBuilder:
        """Append a validated waypoint specification."""

        return self.use(
            "waypoint",
            waypoint.id,
            duration_s=waypoint.duration_s,
            target=waypoint.target,
            tolerance=waypoint.tolerance,
            dwell_time_s=waypoint.dwell_time_s,
            controller=waypoint.controller,
            actuator_binding=waypoint.actuator_binding,
            reference=waypoint.reference,
        )
        ####

    def waypoint_course(self, waypoints: Iterable[WaypointSpec]) -> TrajectoryBuilder:
        """Append a sequence of waypoints in declaration order."""

        for waypoint in waypoints:
            self.waypoint(waypoint)
        return self
        ####

    def build(self) -> SegmentationScenario:
        """Finalize the sequential graph and return a validated scenario."""

        if not self._segments:
            raise ValueError("trajectory requires at least one segment")
        segments: list[SegmentSpec] = []
        for index, segment in enumerate(self._segments):
            is_last = index == len(self._segments) - 1
            exit_spec = segment.exit.model_copy(
                update={
                    "target": None if is_last else self._segments[index + 1].id,
                    "action": "stop" if is_last else "goto",
                }
            )
            segments.append(segment.model_copy(update={"exit": exit_spec}))
        scenario = SegmentationScenario(
            id=self.scenario_id,
            vehicle=self.vehicle,
            family=self.family,
            source_problem=self.source_problem,
            output_problem=self.output_problem,
            output_manifest=self.output_manifest,
            output_audit=self.output_audit,
            runtime_tables=self.runtime_tables,
            segments=tuple(segments),
        )
        evaluate_composition(scenario).raise_for_failure()
        return scenario
        ####

    def evaluate(self) -> CompositionReport:
        """Build and evaluate the current composition."""

        return evaluate_composition(self.build())
        ####

    def compile(self, root: Path) -> tuple[Path, Path, Path]:
        """Build and lower the composition through the existing segment compiler."""

        from taoryx.segmentation import compile_scenario

        return compile_scenario(self.build(), root)
        ####


def _segment_end_time(segment: SegmentSpec) -> float:
    """Extract a builder time from a standard template segment."""

    condition = segment.exit.condition.strip()
    if not condition.startswith("time > "):
        raise ValueError(f"segment {segment.id!r} is not a standard time-windowed component")
    return float(condition.removeprefix("time > "))
    ####


_TIME_ENTRY = re.compile(r"^time\s*>=\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)$")
_TIME_EXIT = re.compile(r"^time\s*>\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?[0-9]+)?)$")
_TRANSITION_CHANNEL_GROUPS: dict[str, tuple[str, ...]] = {
    "position": ("position.", "position_", "x", "y", "z"),
    "velocity": ("velocity.", "velocity_", "speed", "xdt", "ydt", "zdt"),
    "attitude": ("attitude.", "attitude_", "heading", "pitch", "roll"),
    "rates": ("rate.", "rates.", "rate_", "wx", "wy", "wz"),
    "mass": ("mass.", "mass_", "wt"),
}
_SATURATION_CHANNELS = (
    "actuator_saturated",
    "control.saturated",
    "control.saturation",
    "actuator.saturation",
)


def evaluate_segment_runtime(
    segment: SegmentSpec,
    artifact: RunArtifact,
    *,
    vehicle_id: str,
    segment_index: int = 0,
    options: RuntimeEvaluationOptions | None = None,
) -> SegmentRuntimeEvaluation:
    """Evaluate one segment against aligned telemetry in a :class:`RunArtifact`."""

    selected_options = options or RuntimeEvaluationOptions()
    vehicle = _require_vehicle(artifact, vehicle_id)
    spans = _runtime_spans(vehicle, (segment,), selected_options)
    span = spans[segment_index] if segment_index < len(spans) else None
    indices = _span_indices(vehicle, span) if span is not None else []
    entry, goal, transition, exit_checks, quality, messages = _evaluate_runtime_parts(
        segment,
        vehicle,
        indices,
        span,
        None,
        None,
        _transition_events(vehicle, artifact),
        selected_options,
        segment_index,
        1,
        termination=artifact.termination,
    )
    return _runtime_segment_result(segment, vehicle, indices, span, entry, goal, transition, exit_checks, quality, messages)
    ####


def evaluate_composition_runtime(
    scenario: SegmentationScenario,
    artifact: RunArtifact,
    *,
    vehicle_id: str | None = None,
    options: RuntimeEvaluationOptions | None = None,
) -> RuntimeCompositionReport:
    """Evaluate every composed segment against one recorded vehicle run.

    Artifact segment spans are preferred.  Standard time windows may be
    inferred when spans were not emitted, but that fact is reported as a
    warning and non-time segments are blocked rather than guessed.
    """

    selected_options = options or RuntimeEvaluationOptions()
    selected_vehicle_id = vehicle_id or scenario.vehicle
    vehicle = _require_vehicle(artifact, selected_vehicle_id)
    spans = _runtime_spans(vehicle, scenario.segments, selected_options)
    evaluations: list[SegmentRuntimeEvaluation] = []
    events = _transition_events(vehicle, artifact)
    for index, segment in enumerate(scenario.segments):
        span = spans[index] if index < len(spans) else None
        indices = _span_indices(vehicle, span) if span is not None else []
        previous_span = spans[index - 1] if index > 0 and index - 1 < len(spans) else None
        next_span = spans[index + 1] if index + 1 < len(spans) else None
        previous_indices = _span_indices(vehicle, previous_span) if previous_span is not None else []
        next_indices = _span_indices(vehicle, next_span) if next_span is not None else []
        entry, goal, transition, exit_checks, quality, messages = _evaluate_runtime_parts(
            segment,
            vehicle,
            indices,
            span,
            previous_span,
            next_span,
            events,
            selected_options,
            index,
            len(scenario.segments),
            termination=artifact.termination,
            previous_indices=previous_indices,
            next_indices=next_indices,
        )
        evaluations.append(_runtime_segment_result(segment, vehicle, indices, span, entry, goal, transition, exit_checks, quality, messages))
    checks = _composition_runtime_checks(scenario, vehicle, spans, evaluations, explicit_spans=bool(vehicle.segments))
    statuses = [evaluation.status for evaluation in evaluations]
    if any(status == "fail" for status in statuses) or any(check.status == "fail" for check in checks):
        status: RuntimeStatus = "fail"
    elif any(status == "blocked" for status in statuses) or any(check.status == "blocked" for check in checks):
        status = "blocked"
    elif any(status == "warning" for status in statuses) or any(check.status == "warning" for check in checks):
        status = "warning"
    else:
        status = "pass"
    failure_messages = tuple(
        f"{evaluation.segment_id}: {message}"
        for evaluation in evaluations
        for message in evaluation.messages
        if evaluation.status in {"fail", "blocked"}
    )
    return RuntimeCompositionReport(
        scenario_id=scenario.id,
        vehicle_id=selected_vehicle_id,
        status=status,
        segments=tuple(evaluations),
        checks=tuple(checks),
        messages=failure_messages,
    )
    ####


def evaluate_run(
    scenario: SegmentationScenario,
    artifact: RunArtifact,
    *,
    vehicle_id: str | None = None,
    options: RuntimeEvaluationOptions | None = None,
) -> RuntimeCompositionReport:
    """Short alias for agents evaluating a composed trajectory run."""

    return evaluate_composition_runtime(scenario, artifact, vehicle_id=vehicle_id, options=options)
    ####


def _require_vehicle(artifact: RunArtifact, vehicle_id: str) -> VehicleTelemetry:
    try:
        return artifact.vehicles[vehicle_id]
    except KeyError as error:
        raise ValueError(f"run artifact does not contain vehicle {vehicle_id!r}") from error
    ####


def _runtime_spans(
    vehicle: VehicleTelemetry,
    segments: tuple[SegmentSpec, ...],
    options: RuntimeEvaluationOptions,
) -> tuple[SegmentSpan, ...]:
    if vehicle.segments:
        return tuple(vehicle.segments)
    if options.require_explicit_segment_spans:
        return ()
    inferred: list[SegmentSpan] = []
    for number, segment in enumerate(segments, start=1):
        start_match = _TIME_ENTRY.fullmatch(segment.entry.condition.strip())
        end_match = _TIME_EXIT.fullmatch(segment.exit.condition.strip())
        if start_match is None or end_match is None:
            return ()
        inferred.append(SegmentSpan(number=number, title=segment.id, start_time=float(start_match.group(1)), end_time=float(end_match.group(1))))
    return tuple(inferred)
    ####


def _span_indices(vehicle: VehicleTelemetry, span: SegmentSpan | None) -> list[int]:
    if span is None:
        return []
    return [index for index, time in enumerate(vehicle.times) if span.start_time <= time <= span.end_time]
    ####


def _transition_events(vehicle: VehicleTelemetry, artifact: RunArtifact) -> tuple[dict[str, object], ...]:
    events: list[dict[str, object]] = [event.model_dump(mode="python") for event in vehicle.events]
    events.extend(dict(event) for event in artifact.events if isinstance(event, Mapping))
    return tuple(events)
    ####


def _evaluate_runtime_parts(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    span: SegmentSpan | None,
    previous_span: SegmentSpan | None,
    next_span: SegmentSpan | None,
    events: tuple[dict[str, object], ...],
    options: RuntimeEvaluationOptions,
    segment_index: int,
    segment_count: int,
    *,
    termination: Mapping[str, object] | None = None,
    previous_indices: list[int] | None = None,
    next_indices: list[int] | None = None,
) -> tuple[list[RuntimeCheck], list[RuntimeCheck], list[RuntimeCheck], list[RuntimeCheck], list[RuntimeCheck], list[str]]:
    entry = _entry_checks(segment, vehicle, indices, options)
    goal = _goal_checks(segment, vehicle, indices, options)
    transition = _transition_checks(segment, vehicle, indices, previous_span, span, previous_indices or [], events, options, segment_index)
    exit_checks = _exit_checks(segment, vehicle, indices, span, next_span, next_indices or [], segment_index, segment_count, termination or {})
    quality = _quality_checks(vehicle, indices, segment, options)
    messages = [check.message for check in (*entry, *goal, *transition, *exit_checks, *quality) if check.message and check.status in {"fail", "blocked"}]
    return entry, goal, transition, exit_checks, quality, messages
    ####


def _runtime_segment_result(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    span: SegmentSpan | None,
    entry: list[RuntimeCheck],
    goal: list[RuntimeCheck],
    transition: list[RuntimeCheck],
    exit_checks: list[RuntimeCheck],
    quality: list[RuntimeCheck],
    messages: list[str],
) -> SegmentRuntimeEvaluation:
    checks = (*entry, *goal, *transition, *exit_checks, *quality)
    if any(check.status == "fail" for check in checks):
        status: RuntimeStatus = "fail"
    elif any(check.status == "blocked" for check in checks):
        status = "blocked"
    elif any(check.status == "warning" for check in checks):
        status = "warning"
    else:
        status = "pass"
    return SegmentRuntimeEvaluation(
        segment_id=segment.id,
        status=status,
        start_time_s=None if span is None else span.start_time,
        end_time_s=None if span is None else span.end_time,
        sample_count=len(indices),
        entry_checks=tuple(entry),
        goal_checks=tuple(goal),
        transition_checks=tuple(transition),
        exit_checks=tuple(exit_checks),
        quality_checks=tuple(quality),
        messages=tuple(messages),
    )
    ####


def _entry_checks(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    options: RuntimeEvaluationOptions,
) -> list[RuntimeCheck]:
    if not indices:
        return [RuntimeCheck(name="entry-window", status="blocked", message="segment has no telemetry samples")]
    if not segment.initial_state:
        return [RuntimeCheck(name="entry-state", status="pass", message="no explicit initial state was declared")]
    checks: list[RuntimeCheck] = []
    first = indices[0]
    for requested_name, expected in segment.initial_state.items():
        channel = _resolve_channel(vehicle, requested_name, options.channel_aliases)
        if channel is None:
            checks.append(RuntimeCheck(name=f"entry:{requested_name}", status="blocked", expected=expected, message=f"required entry channel {requested_name!r} is unavailable"))
            continue
        actual = channel.values[first]
        if actual is None or not math.isfinite(float(actual)):
            checks.append(RuntimeCheck(name=f"entry:{requested_name}", status="blocked", actual=actual, expected=expected, message=f"entry channel {requested_name!r} is not finite"))
            continue
        error = abs(float(actual) - float(expected))
        checks.append(
            RuntimeCheck(
                name=f"entry:{requested_name}",
                status="pass" if error <= options.entry_tolerance else "fail",
                actual=float(actual),
                expected=float(expected),
                tolerance=options.entry_tolerance,
                message=None if error <= options.entry_tolerance else f"entry channel {requested_name!r} differs from the declared initial state",
            )
        )
    return checks
    ####


def _goal_checks(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    options: RuntimeEvaluationOptions,
) -> list[RuntimeCheck]:
    goal = segment.goal
    if goal is None:
        return [RuntimeCheck(name="goal", status="warning", message="segment has no typed goal")]
    if not goal.target:
        return [RuntimeCheck(name=f"goal:{goal.id}", status="warning", message="goal has no numeric target; capture was not evaluated")]
    if not indices:
        return [RuntimeCheck(name=f"goal:{goal.id}", status="blocked", message="goal window has no telemetry samples")]
    checks: list[RuntimeCheck] = []
    in_band: list[list[bool]] = []
    resolved: list[tuple[str, object, float, list[float]]] = []
    for requested_name, target in goal.target.items():
        tolerance = goal.tolerance.get(requested_name)
        if tolerance is None or tolerance <= 0.0:
            checks.append(RuntimeCheck(name=f"goal:{requested_name}:contract", status="blocked", expected=tolerance, message=f"goal channel {requested_name!r} has no positive tolerance"))
            continue
        channel = _resolve_channel(vehicle, requested_name, options.channel_aliases)
        if channel is None:
            checks.append(RuntimeCheck(name=f"goal:{requested_name}:channel", status="blocked", expected=requested_name, message=f"required goal channel {requested_name!r} is unavailable"))
            continue
        values: list[float] = []
        invalid = False
        for index in indices:
            value = channel.values[index]
            if value is None or not math.isfinite(float(value)):
                invalid = True
                break
            values.append(float(value))
        if invalid:
            checks.append(RuntimeCheck(name=f"goal:{requested_name}:finite", status="blocked", message=f"goal channel {requested_name!r} contains missing or non-finite telemetry"))
            continue
        resolved.append((requested_name, channel, float(target), values))
        in_band.append([_within_tolerance(value, float(target), float(tolerance), channel) for value in values])
    if not resolved:
        return checks or [RuntimeCheck(name=f"goal:{goal.id}", status="blocked", message="goal has no evaluable channels")]
    final_capture = all(flags[-1] for flags in in_band)
    trailing_dwell = _trailing_dwell(vehicle.times, indices, in_band)
    required_dwell = float(goal.dwell_time_s)
    for (requested_name, channel, target, values), flags in zip(resolved, in_band, strict=True):
        tolerance = float(goal.tolerance[requested_name])
        final_error = _error(values[-1], target, channel)
        checks.append(
            RuntimeCheck(
                name=f"goal:{requested_name}:final",
                status="pass" if flags[-1] else "fail",
                actual=values[-1],
                expected=target,
                tolerance=tolerance,
                message=None if flags[-1] else f"final value for goal channel {requested_name!r} is outside tolerance (error={final_error:g})",
            )
        )
    dwell_status = final_capture and trailing_dwell + 1.0e-12 >= required_dwell
    checks.append(
        RuntimeCheck(
            name=f"goal:{goal.id}:dwell",
            status="pass" if dwell_status else "fail",
            actual=trailing_dwell,
            expected=required_dwell,
            tolerance=0.0,
            message=None if dwell_status else f"goal dwell was {trailing_dwell:g}s; required {required_dwell:g}s at the segment exit",
        )
    )
    if goal.completion_condition:
        checks.append(RuntimeCheck(name=f"goal:{goal.id}:completion-condition", status="warning", expected=goal.completion_condition, message="completion_condition is preserved but runtime expression evaluation is not attached to this artifact evaluator"))
    return checks
    ####


def _transition_checks(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    previous_span: SegmentSpan | None,
    span: SegmentSpan | None,
    previous_indices: list[int],
    events: tuple[dict[str, object], ...],
    options: RuntimeEvaluationOptions,
    segment_index: int,
) -> list[RuntimeCheck]:
    if segment_index == 0:
        return [RuntimeCheck(name="transition:first-segment", status="pass", message="no incoming transition is required")]
    if previous_span is None or span is None or not previous_indices or not indices:
        return [RuntimeCheck(name="transition:boundary", status="blocked", message="incoming transition boundary is unavailable")]
    event_found = any(
        event.get("segment_from") == previous_span.number and event.get("segment_to") == span.number
        for event in events
    )
    checks = [
        RuntimeCheck(
            name="transition:event",
            status="pass" if event_found else "warning",
            actual=event_found,
            expected=True,
            message=None if event_found else "no recorded segment-transition event matched this boundary",
        )
    ]
    for requested_name, tolerance in options.transition_tolerances.items():
        channel = _resolve_channel(vehicle, requested_name, options.channel_aliases)
        if channel is None:
            checks.append(RuntimeCheck(name=f"transition:{requested_name}", status="blocked", expected=requested_name, message=f"transition channel {requested_name!r} is unavailable"))
            continue
        left_index = previous_indices[-1]
        right_index = indices[0] if indices[0] > left_index else (indices[1] if len(indices) > 1 else indices[0])
        left = channel.values[left_index]
        right = channel.values[right_index]
        if left is None or right is None or not math.isfinite(float(left)) or not math.isfinite(float(right)):
            checks.append(RuntimeCheck(name=f"transition:{requested_name}", status="blocked", message=f"transition channel {requested_name!r} is not finite at the boundary"))
            continue
        policy = _policy_for_channel(segment.transition, requested_name)
        jump = _error(float(right), float(left), channel)
        allowed = policy not in {"continuous", "inherit"} or jump <= tolerance
        checks.append(
            RuntimeCheck(
                name=f"transition:{requested_name}",
                status="pass" if allowed else "fail",
                actual=jump,
                expected=policy,
                tolerance=tolerance,
                message=None if allowed else f"declared {policy} transition for {requested_name!r} exceeded tolerance",
            )
        )
    if not options.transition_tolerances:
        checks.append(RuntimeCheck(name="transition:numeric-continuity", status="not_run", message="provide transition_tolerances to audit numeric handoff jumps"))
    return checks
    ####


def _exit_checks(
    segment: SegmentSpec,
    vehicle: VehicleTelemetry,
    indices: list[int],
    span: SegmentSpan | None,
    next_span: SegmentSpan | None,
    next_indices: list[int],
    segment_index: int,
    segment_count: int,
    termination: Mapping[str, object],
) -> list[RuntimeCheck]:
    if span is None or not indices:
        return [RuntimeCheck(name="exit:window", status="blocked", message="segment exit has no recorded telemetry window")]
    if segment_index < segment_count - 1:
        if next_span is None or not next_indices:
            return [RuntimeCheck(name="exit:next-segment", status="blocked", expected=segment.exit.target, message=f"expected next segment {segment.exit.target!r} was not recorded")]
        contiguous = next_span.start_time <= span.end_time + _time_tolerance(vehicle)
        return [RuntimeCheck(name="exit:next-segment", status="pass" if contiguous else "fail", actual=next_span.start_time, expected=span.end_time, tolerance=_time_tolerance(vehicle), message=None if contiguous else "next segment does not begin at the declared handoff")]
    terminal = abs(span.end_time - vehicle.times[-1]) <= _time_tolerance(vehicle)
    completed = termination.get("completed")
    stop_reason = termination.get("stop_reason")
    if completed is False:
        termination_check = RuntimeCheck(name="exit:termination-reason", status="fail", actual=stop_reason, expected="completed", message=f"run did not complete normally (stop_reason={stop_reason!r})")
    elif completed is True and stop_reason is not None:
        termination_check = RuntimeCheck(name="exit:termination-reason", status="pass", actual=stop_reason, expected="recorded stop reason")
    else:
        termination_check = RuntimeCheck(name="exit:termination-reason", status="warning", message="RunArtifact does not include a typed stop reason; terminal coverage was checked instead")
    return [
        RuntimeCheck(
            name="exit:terminal-coverage",
            status="pass" if terminal else "fail",
            actual=span.end_time,
            expected=vehicle.times[-1],
            tolerance=_time_tolerance(vehicle),
            message=None if terminal else "terminal segment does not cover the final telemetry sample",
        ),
        termination_check,
    ]
    ####


def _quality_checks(
    vehicle: VehicleTelemetry,
    indices: list[int],
    segment: SegmentSpec,
    options: RuntimeEvaluationOptions,
) -> list[RuntimeCheck]:
    if not indices:
        return [RuntimeCheck(name="quality:sample-count", status="blocked", actual=0, expected=1, message="quality checks require at least one telemetry sample")]
    requested = (
        set(segment.initial_state)
        | (set(segment.goal.target) if segment.goal is not None else set())
        | set(options.required_channels)
    )
    missing: list[str] = []
    for name in requested:
        channel = _resolve_channel(vehicle, name, options.channel_aliases)
        if channel is None or any(channel.values[index] is None or not math.isfinite(float(channel.values[index])) for index in indices):
            missing.append(name)
    checks = [RuntimeCheck(name="quality:finite-telemetry", status="blocked" if missing else "pass", actual=missing or True, expected=True, message=f"missing or non-finite required channels: {', '.join(sorted(missing))}" if missing else None)]
    for name in options.required_channels:
        checks.append(
            RuntimeCheck(
                name=f"quality:required-channel:{name}",
                status="blocked" if name in missing else "pass",
                expected=name,
                message=f"required runtime channel {name!r} is unavailable or non-finite" if name in missing else None,
            )
        )
    saturation = _find_channel(vehicle, _SATURATION_CHANNELS)
    if saturation is None:
        if options.max_saturation_fraction is not None:
            checks.append(RuntimeCheck(name="quality:saturation", status="blocked", expected=options.max_saturation_fraction, message="a saturation limit was requested but no saturation channel was emitted"))
        else:
            checks.append(RuntimeCheck(name="quality:saturation", status="not_run", message="no actuator saturation channel was emitted"))
    else:
        values = [saturation.values[index] for index in indices]
        fraction = sum(value is not None and float(value) != 0.0 for value in values) / len(values)
        limit = options.max_saturation_fraction
        checks.append(RuntimeCheck(name="quality:saturation", status="pass" if limit is None or fraction <= limit else "fail", actual=fraction, expected=limit, message=None if limit is None or fraction <= limit else "actuator saturation fraction exceeded the configured limit"))
    return checks
    ####


def _composition_runtime_checks(
    scenario: SegmentationScenario,
    vehicle: VehicleTelemetry,
    spans: tuple[SegmentSpan, ...],
    evaluations: list[SegmentRuntimeEvaluation],
    *,
    explicit_spans: bool,
) -> list[RuntimeCheck]:
    checks: list[RuntimeCheck] = []
    if not spans:
        checks.append(RuntimeCheck(name="composition:segment-spans", status="blocked", message="segment spans were not emitted and the scenario has no reconstructable time windows"))
    elif len(spans) != len(scenario.segments):
        checks.append(RuntimeCheck(name="composition:segment-count", status="fail", actual=len(spans), expected=len(scenario.segments), message="recorded segment count does not match the composed scenario"))
    elif not explicit_spans:
        checks.append(RuntimeCheck(name="composition:segment-spans", status="warning", message="segment windows were inferred from time entry/exit conditions"))
    else:
        checks.append(RuntimeCheck(name="composition:segment-spans", status="pass", actual=len(spans), expected=len(scenario.segments)))
    checks.append(RuntimeCheck(name="composition:terminal-stop", status="pass" if scenario.segments[-1].exit.action == "stop" else "fail", expected="stop", actual=scenario.segments[-1].exit.action, message=None if scenario.segments[-1].exit.action == "stop" else "scenario is not terminally configured"))
    return checks
    ####


def _resolve_channel(vehicle: VehicleTelemetry, requested: str, aliases: Mapping[str, str]) -> Any | None:
    mapped = aliases.get(requested, requested)
    return _find_channel(vehicle, (mapped, requested))
    ####


def _find_channel(vehicle: VehicleTelemetry, names: Iterable[str]) -> Any | None:
    candidates = {str(name).casefold() for name in names}
    for name, channel in vehicle.channels.items():
        if name.casefold() in candidates or channel.semantic_name.casefold() in candidates or channel.source_name.casefold() in candidates:
            return channel
    return None
    ####


def _within_tolerance(actual: float, target: float, tolerance: float, channel: Any) -> bool:
    return _error(actual, target, channel) <= tolerance
    ####


def _error(actual: float, target: float, channel: Any) -> float:
    if getattr(channel, "interpolation", "linear") == "angle":
        return abs((actual - target + 180.0) % 360.0 - 180.0)
    return abs(actual - target)
    ####


def _trailing_dwell(times: list[float], indices: list[int], flags: list[list[bool]]) -> float:
    if not indices or not flags or not all(series[-1] for series in flags):
        return 0.0
    start = len(indices) - 1
    while start > 0 and all(series[start - 1] and series[start] for series in flags):
        start -= 1
    return times[indices[-1]] - times[indices[start]]
    ####


def _policy_for_channel(policy: TransitionPolicy, channel: str) -> str:
    lowered = channel.casefold()
    for group, prefixes in _TRANSITION_CHANNEL_GROUPS.items():
        if any(lowered.startswith(prefix) or prefix in lowered for prefix in prefixes):
            return str(getattr(policy, group))
    return "continuous"
    ####


def _time_tolerance(vehicle: VehicleTelemetry) -> float:
    if len(vehicle.times) < 2:
        return 1.0e-9
    return max(1.0e-9, 1.5 * max(right - left for left, right in zip(vehicle.times, vehicle.times[1:], strict=False)))
    ####


def evaluate_segment(segment: SegmentSpec) -> SegmentEvaluation:
    """Evaluate one segment's structural contract."""

    checks: list[str] = []
    messages: list[str] = []
    status: CompositionStatus = "pass"
    if segment.goal is None:
        status = "warning"
        messages.append("segment has no typed goal")
    else:
        checks.append(f"goal:{segment.goal.kind}")
        if segment.goal.kind in {"waypoint", "altitude_capture", "heading_capture", "intercept_geometry", "terminal_guidance"} and not segment.goal.target:
            status = "fail"
            messages.append("capture segment requires a non-empty target")
        if segment.goal.kind in {"waypoint", "altitude_capture", "heading_capture", "intercept_geometry", "terminal_guidance"} and not segment.goal.tolerance:
            status = "fail"
            messages.append("capture segment requires a non-empty tolerance")
        if segment.goal.kind in {"intercept_geometry", "terminal_guidance"} and not segment.goal.reference:
            status = "fail"
            messages.append("terminal guidance requires a target reference")
    if segment.controller is None:
        status = "warning" if status == "pass" else status
        messages.append("no controller binding declared")
    if segment.actuator_binding is None:
        status = "warning" if status == "pass" else status
        messages.append("no actuator binding declared")
    checks.extend(("entry:declared", "exit:declared", f"mode:{segment.mode}"))
    return SegmentEvaluation(segment_id=segment.id, status=status, checks=tuple(checks), messages=tuple(messages))
    ####


def evaluate_composition(scenario: SegmentationScenario) -> CompositionReport:
    """Evaluate the graph and report warnings before compilation or execution."""

    evaluations = tuple(evaluate_segment(segment) for segment in scenario.segments)
    messages: list[str] = []
    checks = ["unique segment IDs", "valid entry/exit graph", "terminal stop"]
    if scenario.segments[-1].exit.action != "stop":
        messages.append("composition must terminate with a stop segment")
    for evaluation in evaluations:
        messages.extend(
            f"{evaluation.segment_id}: {message}"
            for message in evaluation.messages
            if evaluation.status == "fail"
        )
    if messages:
        status: CompositionStatus = "fail"
    elif any(evaluation.status == "warning" for evaluation in evaluations):
        status = "warning"
    else:
        status = "pass"
    return CompositionReport(
        scenario_id=scenario.id,
        status=status,
        segments=evaluations,
        checks=tuple(checks),
        messages=tuple(messages),
    )
    ####
