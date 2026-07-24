"""Declarative mission segmentation outside the historical TAOS language.

The catalog is orchestration metadata.  Compilation emits ordinary TAOS/TAORYX
problem syntax and never adds a segmentation directive to the historical
language.  This keeps reusable mission structure easy to author while keeping
the parser's compatibility boundary honest.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from taoryx.control import SegmentPlan, SegmentSchedule
from taoryx.language.expressions import ExpressionSyntaxError, parse_expression


class SegmentationCompileError(ValueError):
    """Raised when a declarative segment catalog cannot be safely lowered."""


DynamicsMode = Literal["point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"]
ContinuityPolicy = Literal["continuous", "inherit", "reset", "impulse", "mass_change"]
GoalKind = Literal[
    "trim_hold",
    "hover",
    "altitude_capture",
    "heading_capture",
    "waypoint",
    "racetrack",
    "separation",
    "ground_contact",
    "intercept_geometry",
    "powered_ascent",
    "ballistic_coast",
    "bank_maneuver",
    "alpha_maneuver",
    "skip_maneuver",
    "terminal_guidance",
]
EventKind = Literal[
    "time",
    "altitude",
    "mach",
    "fuel_remaining",
    "signal",
    "waypoint_reached",
    "stage_burnout",
    "separation",
    "ground_contact",
    "envelope_exit",
]


class TransitionPolicy(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    position: ContinuityPolicy = "continuous"
    velocity: ContinuityPolicy = "continuous"
    attitude: ContinuityPolicy = "continuous"
    rates: ContinuityPolicy = "continuous"
    mass: ContinuityPolicy = "continuous"


class GoalSpec(BaseModel):
    """Typed acceptance goal for a segment or scenario.

    Goals are evidence metadata, not executable TAOS syntax.  ``target`` and
    ``tolerance`` use canonical channel names (for example ``altitude_m`` or
    ``heading_deg``), allowing the same contract to serve point-mass, bridge,
    and rigid-body adapters without embedding a vehicle-specific state class.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1, pattern=r"^[A-Za-z][A-Za-z0-9_-]*$")
    kind: GoalKind
    target: Mapping[str, float] = Field(default_factory=dict)
    tolerance: Mapping[str, float] = Field(default_factory=dict)
    dwell_time_s: float = Field(default=0.0, ge=0.0)
    reference: str | None = None
    completion_condition: str | None = None

    @field_validator("target", "tolerance")
    @classmethod
    def finite_values(cls, value: Mapping[str, float]) -> Mapping[str, float]:
        if any(not name.strip() for name in value):
            raise ValueError("goal channel names cannot be empty")
        if any(not math.isfinite(float(item)) for item in value.values()):
            raise ValueError("goal target and tolerance values must be finite")
        return value
        ####

    @field_validator("tolerance")
    @classmethod
    def positive_tolerances(cls, value: Mapping[str, float]) -> Mapping[str, float]:
        if any(float(item) <= 0.0 for item in value.values()):
            raise ValueError("goal tolerances must be positive")
        return value
        ####

    @field_validator("completion_condition")
    @classmethod
    def valid_completion_condition(cls, value: str | None) -> str | None:
        if value is not None:
            try:
                parse_expression(value)
            except (ExpressionSyntaxError, ValueError) as error:
                raise ValueError(f"invalid goal completion condition: {value!r}") from error
        return value
        ####


class TransitionEventSpec(BaseModel):
    """Typed event metadata for segment changes and declared discontinuities."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1, pattern=r"^[A-Za-z][A-Za-z0-9_-]*$")
    kind: EventKind
    condition: str = Field(min_length=1)
    action: Literal["continue", "goto", "stop", "emit"] = "emit"
    target: str | None = None
    transition: TransitionPolicy = TransitionPolicy()
    mass_delta_kg: float | None = None
    impulse_body_mps: tuple[float, float, float] | None = None
    signal: str | None = None
    reason: str | None = None

    @field_validator("condition")
    @classmethod
    def valid_condition(cls, value: str) -> str:
        try:
            parse_expression(value)
        except (ExpressionSyntaxError, ValueError) as error:
            raise ValueError(f"invalid transition event condition: {value!r}") from error
        return value
        ####

    @field_validator("mass_delta_kg")
    @classmethod
    def finite_mass_delta(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("event mass_delta_kg must be finite")
        return value
        ####

    @field_validator("impulse_body_mps")
    @classmethod
    def finite_impulse(cls, value: tuple[float, float, float] | None) -> tuple[float, float, float] | None:
        if value is not None and any(not math.isfinite(item) for item in value):
            raise ValueError("event impulse_body_mps must be finite")
        return value
        ####

    @model_validator(mode="after")
    def validate_action(self) -> TransitionEventSpec:
        if self.action == "goto" and self.target is None:
            raise ValueError(f"event {self.id!r} with action goto requires a target")
        if self.action != "goto" and self.target is not None:
            raise ValueError(f"event {self.id!r} has a target but action is {self.action!r}")
        return self
        ####


class SegmentEntry(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    condition: str = Field(min_length=1)
    frame: str | None = None
    units: str | None = None

    @field_validator("frame")
    @classmethod
    def known_frame(cls, value: str | None) -> str | None:
        if value is not None and value.casefold() not in {"ecfc", "ecic", "body", "ned", "geodetic", "source"}:
            raise ValueError(f"unsupported segment frame {value!r}")
        return value
        ####

    @field_validator("units")
    @classmethod
    def known_units(cls, value: str | None) -> str | None:
        if value is not None and value.casefold() not in {"si", "fps", "us_customary", "native"}:
            raise ValueError(f"unsupported segment unit profile {value!r}")
        return value
        ####


class SegmentExit(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    condition: str = Field(min_length=1)
    target: str | None = None
    action: Literal["goto", "stop"] = "goto"


class SegmentSpec(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1, pattern=r"^[A-Za-z][A-Za-z0-9_-]*$")
    source_segment: int = Field(default=1, ge=1)
    mode: DynamicsMode
    entry: SegmentEntry
    exit: SegmentExit
    controller: str | None = None
    actuator_binding: str | None = None
    controls: tuple[str, ...] = ()
    initial_state: Mapping[str, float] = Field(default_factory=dict)
    inherited_from: str | None = None
    transition_values: Mapping[str, float] = Field(default_factory=dict)
    table_dependencies: tuple[str, ...] = ()
    environment_dependencies: tuple[str, ...] = ()
    transition: TransitionPolicy = TransitionPolicy()
    goal: GoalSpec | None = None

    @model_validator(mode="after")
    def state_source_is_unambiguous(self) -> SegmentSpec:
        if self.initial_state and self.inherited_from is not None:
            raise ValueError(f"segment {self.id!r} cannot provide both initial_state and inherited_from")
        if any(not name.strip() for name in self.controls):
            raise ValueError(f"segment {self.id!r} contains an empty control binding")
        if self.inherited_from == self.id:
            raise ValueError(f"segment {self.id!r} cannot inherit from itself")
        for label, condition in (("entry", self.entry.condition), ("exit", self.exit.condition)):
            try:
                parse_expression(condition)
            except (ExpressionSyntaxError, ValueError) as error:
                raise ValueError(f"segment {self.id!r} has invalid {label} condition: {condition!r}") from error
        return self
        ####


class SegmentationScenario(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    vehicle: str = Field(min_length=1)
    family: str = Field(min_length=1)
    source_problem: str = Field(min_length=1)
    output_problem: str = Field(min_length=1)
    output_manifest: str = Field(min_length=1)
    output_audit: str = Field(min_length=1)
    runtime_tables: tuple[str, ...] = ()
    segments: tuple[SegmentSpec, ...] = Field(min_length=1)
    events: tuple[TransitionEventSpec, ...] = ()

    @model_validator(mode="after")
    def validate_graph(self) -> SegmentationScenario:
        identifiers = [segment.id for segment in self.segments]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError(f"scenario {self.id!r} has duplicate segment IDs")
        event_ids = [event.id for event in self.events]
        if len(set(event_ids)) != len(event_ids):
            raise ValueError(f"scenario {self.id!r} has duplicate transition event IDs")
        known = set(identifiers)
        known_events = set(event_ids)
        for segment in self.segments:
            target = segment.exit.target
            if target is not None and target not in known:
                raise ValueError(f"segment {segment.id!r} targets missing segment {target!r}")
            if segment.exit.action == "stop" and target is not None:
                raise ValueError(f"stopping segment {segment.id!r} cannot have a target")
            if segment.exit.action == "goto" and target is None:
                raise ValueError(f"goto segment {segment.id!r} requires a target")
            if segment.inherited_from is not None and segment.inherited_from not in known:
                raise ValueError(f"segment {segment.id!r} inherits from missing segment {segment.inherited_from!r}")
        for event in self.events:
            if event.target is not None and event.target not in known:
                raise ValueError(f"event {event.id!r} targets missing segment {event.target!r}")
            if event.signal is not None and not event.signal.strip():
                raise ValueError(f"event {event.id!r} has an empty signal name")
        if known_events.intersection(known):
            raise ValueError(f"scenario {self.id!r} reuses an ID between segment and event")
        graph = {segment.id: segment.exit.target for segment in self.segments if segment.exit.target}
        incoming: dict[str, list[SegmentSpec]] = {}
        for segment in self.segments:
            if segment.exit.target is not None:
                incoming.setdefault(segment.exit.target, []).append(segment)
        for target, sources in incoming.items():
            if len(sources) > 1 and any(source.transition_values for source in sources):
                raise ValueError(f"segment {target!r} has multiple state-changing incoming transitions")
        visiting: set[str] = set()
        visited: set[str] = set()

        def visit(identifier: str) -> None:
            if identifier in visiting:
                raise ValueError(f"scenario {self.id!r} contains a segment cycle at {identifier!r}")
            if identifier in visited:
                return
            visiting.add(identifier)
            target = graph.get(identifier)
            if target is not None:
                visit(target)
            visiting.remove(identifier)
            visited.add(identifier)
        ####

        for identifier in graph:
            visit(identifier)
        return self
        ####


class SegmentationCatalog(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = Field(ge=1)
    id: str = Field(min_length=1)
    description: str = ""
    controllers: tuple[str, ...] = ()
    actuators: tuple[str, ...] = ()
    tables: tuple[str, ...] = ()
    environments: tuple[str, ...] = ()
    scenarios: tuple[SegmentationScenario, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def unique_scenarios(self) -> SegmentationCatalog:
        ids = [scenario.id for scenario in self.scenarios]
        if len(set(ids)) != len(ids):
            raise ValueError("segmentation catalog scenario IDs must be unique")
        for scenario in self.scenarios:
            for segment in scenario.segments:
                if segment.controller is not None and segment.controller not in self.controllers:
                    raise ValueError(f"segment {segment.id!r} references unknown controller {segment.controller!r}")
                if segment.actuator_binding is not None and segment.actuator_binding not in self.actuators:
                    raise ValueError(f"segment {segment.id!r} references unknown actuator {segment.actuator_binding!r}")
                unknown_tables = sorted(set(segment.table_dependencies) - set(self.tables))
                if unknown_tables:
                    raise ValueError(f"segment {segment.id!r} references unknown table(s): {', '.join(unknown_tables)}")
                unknown_environment = sorted(set(segment.environment_dependencies) - set(self.environments))
                if unknown_environment:
                    raise ValueError(
                        f"segment {segment.id!r} references unknown environment(s): {', '.join(unknown_environment)}"
                    )
        return self
        ####


def transition_audit(
    policy: TransitionPolicy,
    before: Mapping[str, float],
    after: Mapping[str, float],
    *,
    tolerance: float = 1e-9,
) -> dict[str, object]:
    """Audit a transition without assuming a vehicle state representation.

    Keys are semantic channels (``position``, ``velocity``, ``attitude``,
    ``rates``, and ``mass``).  The caller supplies scalar summaries or hashes
    for vector channels; the compiler/runtime boundary stays independent of
    dimensionality and frame-specific state classes.
    """

    if tolerance < 0.0:
        raise ValueError("transition audit tolerance must be nonnegative")
    channels = ("position", "velocity", "attitude", "rates", "mass")
    results: dict[str, object] = {}
    violations: list[str] = []
    for channel in channels:
        before_value = before.get(channel)
        after_value = after.get(channel)
        mode = getattr(policy, channel)
        if before_value is None or after_value is None:
            results[channel] = {"policy": mode, "status": "unavailable"}
            continue
        difference = abs(float(after_value) - float(before_value))
        allowed = mode in {"reset", "impulse", "mass_change"} or difference <= tolerance
        if mode == "inherit":
            allowed = difference <= tolerance
        if not allowed:
            violations.append(channel)
        results[channel] = {"policy": mode, "difference": difference, "status": "pass" if allowed else "fail"}
    return {"status": "fail" if violations else "pass", "channels": results, "violations": violations}
    ####


def load_catalog(path: Path) -> SegmentationCatalog:
    """Load and validate an external YAML segmentation catalog."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise SegmentationCompileError(f"catalog is not a mapping: {path}")
    try:
        return SegmentationCatalog.model_validate(payload)
    except ValueError as error:
        raise SegmentationCompileError(str(error)) from error
    ####


def lint_catalog(path: Path, root: Path) -> SegmentationCatalog:
    """Validate a catalog and every referenced native source segment."""

    catalog = load_catalog(path)
    for scenario in catalog.scenarios:
        source_path = root / scenario.source_problem
        if not source_path.is_file():
            raise SegmentationCompileError(f"source problem does not exist: {scenario.source_problem}")
        for table in scenario.runtime_tables:
            if not (root / table).is_file():
                raise SegmentationCompileError(f"runtime table does not exist: {table}")
        source = source_path.read_text(encoding="utf-8")
        for segment in scenario.segments:
            _source_segment_body(source, segment.source_segment)
    return catalog
    ####


_SEGMENT_START = re.compile(r"^  \*segment (\d+) (.+)$")
_END = re.compile(r"^\*end\s*$")
_TIME_ENTRY = re.compile(r"^time\s*(?:>=|>)\s*(-?(?:\d+(?:\.\d*)?|\.\d+))$")
_TIME_EXIT = re.compile(r"^time\s*(?:>|>=|=)\s*(-?(?:\d+(?:\.\d*)?|\.\d+))$")


def _source_segment_body(source: str, segment_number: int) -> tuple[str, ...]:
    lines = source.splitlines()
    starts = [index for index, line in enumerate(lines) if _SEGMENT_START.match(line)]
    if segment_number > len(starts):
        raise SegmentationCompileError(f"source problem has no segment {segment_number}")
    start = starts[segment_number - 1]
    end = next(
        (
            index
            for index in range(start + 1, len(lines))
            if _SEGMENT_START.match(lines[index])
            or lines[index].startswith("*trajectory ")
            or _END.match(lines[index])
        ),
        len(lines),
    )
    body = tuple(lines[start + 1 : end])
    if not any(line.strip().startswith("*integ") for line in body):
        raise SegmentationCompileError(f"source segment {segment_number} has no *integ block")
    return body
    ####


def _native_transition_updates(spec: SegmentSpec) -> tuple[str, ...]:
    """Lower declared transition effects through existing native blocks."""

    if not spec.transition_values:
        return ()
    policy_values = set(
        value
        for value in (spec.transition.position, spec.transition.velocity, spec.transition.attitude, spec.transition.rates, spec.transition.mass)
    )
    if "reset" in policy_values:
        keyword = "reset"
    elif "impulse" in policy_values or "mass_change" in policy_values:
        keyword = "increment"
    else:
        raise SegmentationCompileError(
            f"segment {spec.id!r} declares transition values without reset, impulse, or mass_change policy"
        )
    assignments = " ".join(f"{name}={value}" for name, value in sorted(spec.transition_values.items()))
    return (f"    *{keyword} {assignments}",)
    ####


def _native_initial_updates(spec: SegmentSpec) -> tuple[str, ...]:
    """Lower an explicit segment initialization through native reset syntax."""

    if not spec.initial_state:
        return ()
    assignments = " ".join(f"{name}={value}" for name, value in sorted(spec.initial_state.items()))
    return (f"    *reset {assignments}",)
    ####


def compile_scenario(scenario: SegmentationScenario, root: Path) -> tuple[Path, Path, Path]:
    """Compile one catalog entry and write problem, manifest, and audit files."""

    source_path = root / scenario.source_problem
    if not source_path.is_file():
        raise SegmentationCompileError(f"source problem does not exist: {scenario.source_problem}")
    source = source_path.read_text(encoding="utf-8")
    lines = source.splitlines()
    starts = [index for index, line in enumerate(lines) if _SEGMENT_START.match(line)]
    if not starts:
        raise SegmentationCompileError(f"source problem has no segments: {scenario.source_problem}")
    first = starts[0]
    # Replace only the first trajectory's segment region. Other trajectories
    # (for example a moving target) remain native source content.
    end = next(
        (
            index
            for index in range(first, len(lines))
            if index > first and lines[index].startswith("*trajectory ")
        ),
        next((index for index in range(first, len(lines)) if _END.match(lines[index])), len(lines)),
    )
    prefix = lines[:first]
    suffix = lines[end:]
    ids = {segment.id: index + 1 for index, segment in enumerate(scenario.segments)}
    rendered: list[str] = list(prefix)
    for number, spec in enumerate(scenario.segments, start=1):
        # Keep source-native *when clauses. The catalog transition is an
        # additive orchestration edge; dropping source safety stops would be a
        # silent semantic change.
        body = list(_source_segment_body(source, spec.source_segment))
        rendered.append(f"  *segment {number} {spec.id}")
        if spec.inherited_from is not None:
            # Native segment transitions already inherit the active state. The
            # declaration is retained in the manifest rather than emitting a
            # second, potentially divergent *initial block.
            pass
        incoming = next(
            (candidate for candidate in scenario.segments if candidate.exit.target == spec.id),
            None,
        )
        rendered.extend(_native_transition_updates(incoming) if incoming is not None else ())
        rendered.extend(_native_initial_updates(spec))
        rendered.extend(body)
        if spec.exit.action == "stop":
            rendered.append(f"    *when {spec.exit.condition} stop")
        else:
            rendered.append(f"    *when {spec.exit.condition} goto {ids[spec.exit.target or '']}")
    rendered.extend(suffix)
    output = root / scenario.output_problem
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(rendered).rstrip() + "\n", encoding="utf-8")
    source_hash = hashlib.sha256(source.encode()).hexdigest()
    manifest_payload = {
        "schema_version": 1,
        "scenario_id": scenario.id,
        "vehicle": scenario.vehicle,
        "family": scenario.family,
        "source_problem": scenario.source_problem,
        "runtime_tables": list(scenario.runtime_tables),
        "source_sha256": source_hash,
        "dynamics_modes": [segment.mode for segment in scenario.segments],
        "segments": [segment.model_dump(mode="json") for segment in scenario.segments],
        "goals": [
            {"segment_id": segment.id, "goal": segment.goal.model_dump(mode="json")}
            for segment in scenario.segments
            if segment.goal is not None
        ],
        "events": [event.model_dump(mode="json") for event in scenario.events],
        "historical_syntax": "unchanged; segmentation metadata is external",
        "source_when_clauses_preserved": True,
    }
    manifest = root / scenario.output_manifest
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(manifest_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    audit_payload = {
        "scenario_id": scenario.id,
        "status": "compiled",
        "source_sha256": source_hash,
        "segment_count": len(scenario.segments),
        "source_when_clauses_preserved": True,
        "transitions": [
            {
                "from": scenario.segments[index].id,
                "to": scenario.segments[index].exit.target,
                "policy": scenario.segments[index].transition.model_dump(mode="json"),
                "condition": scenario.segments[index].exit.condition,
            }
            for index in range(len(scenario.segments))
            if scenario.segments[index].exit.action == "goto"
        ],
        "goals": [
            {"segment_id": segment.id, "goal": segment.goal.model_dump(mode="json")}
            for segment in scenario.segments
            if segment.goal is not None
        ],
        "events": [event.model_dump(mode="json") for event in scenario.events],
    }
    audit = root / scenario.output_audit
    audit.parent.mkdir(parents=True, exist_ok=True)
    audit.write_text(json.dumps(audit_payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return output, manifest, audit
    ####


def compile_catalog(path: Path, root: Path) -> tuple[tuple[Path, Path, Path], ...]:
    """Compile all scenarios in a catalog."""

    catalog = load_catalog(path)
    return tuple(compile_scenario(scenario, root) for scenario in catalog.scenarios)
    ####


def time_schedule(scenario: SegmentationScenario) -> SegmentSchedule:
    """Adapt a time-bounded catalog scenario to the controller schedule API.

    This adapter is intentionally strict.  A condition involving altitude,
    table state, or a signal needs the runtime expression evaluator and must
    not be guessed as a time window by orchestration code.
    """

    plans: list[SegmentPlan] = []
    for index, segment in enumerate(scenario.segments):
        entry = _TIME_ENTRY.fullmatch(segment.entry.condition.strip())
        exit_match = _TIME_EXIT.fullmatch(segment.exit.condition.strip())
        if entry is None or exit_match is None:
            raise SegmentationCompileError(
                f"scenario {scenario.id!r} segment {segment.id!r} is not a simple time-bounded segment"
            )
        start = float(entry.group(1))
        end = float(exit_match.group(1))
        if end <= start:
            raise SegmentationCompileError(f"segment {segment.id!r} has non-positive time window")
        plans.append(SegmentPlan(segment.id, start, end, controller_name=segment.controller or ""))
    if plans[0].start_time_s != 0.0:
        raise SegmentationCompileError(f"scenario {scenario.id!r} schedule must start at time zero")
    return SegmentSchedule(tuple(plans))
    ####
