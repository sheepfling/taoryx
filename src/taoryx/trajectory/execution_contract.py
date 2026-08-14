"""Common Mission Composition execution, feedback, and result contracts.

The configuration contract answers what may be requested.  This module
standardizes what a backend runner accepts and returns after configuration:

* structured diagnostics and transport-neutral failure envelopes;
* a discriminated success/failure response;
* a multi-object trajectory with explicit parent/child lineage; and
* a small executor registry that catches provider exceptions at the plug-in
  boundary instead of leaking implementation-specific exceptions to clients.
"""

from __future__ import annotations

import math
import re
from collections.abc import Callable, Mapping
from typing import Annotated, Any, Literal, Protocol, runtime_checkable

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, model_validator

from ..deployment import DeploymentBinding, validate_deployment_bindings
from .configuration_contract import (
    ConfigurableTrajectoryProvider,
    ConfigurationChoiceSchema,
    ConfigurationContractError,
    ConfigurationGroupSchema,
    ConfigurationNode,
    ConfigurationOptionalSchema,
    ConfigurationParameterSchema,
    ConfigurationSequenceSchema,
    PreparedTrajectoryConfiguration,
    TrajectoryControlStatus,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputDataType,
    TrajectoryOutputSchema,
    TrajectorySamplingSemantics,
)

DiagnosticSeverity = Literal["error", "warning", "info"]
DiagnosticPhase = Literal["discovery", "configuration", "preflight", "execution", "projection"]
DiagnosticRecoverability = Literal["correctable", "retryable", "degraded", "fatal"]
FailureCategory = Literal["invalid_request", "unsupported", "unavailable", "execution_failed", "provider_error"]
TrajectoryOutcome = Literal["in_progress", "completed", "terminated", "partial", "failed"]
TrajectoryObjectStatus = Literal["active", "completed", "terminated", "failed"]
TrajectorySegmentStatus = Literal["completed", "terminated", "skipped", "failed"]
_RUN_OPERATIONS: tuple[Literal["batch", "step"], ...] = ("batch", "step")


class MissionCompositionDiagnostic(BaseModel):
    """Stable, machine-actionable feedback from any provider lifecycle phase."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    severity: DiagnosticSeverity
    code: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    message: str = Field(min_length=1)
    phase: DiagnosticPhase
    recoverability: DiagnosticRecoverability
    path: str | None = None
    hint: str | None = None
    provider_id: str | None = None
    model_id: str | None = None
    object_id: str | None = None
    segment_instance_id: str | None = None
    details: dict[str, Any] = Field(default_factory=dict)


class MissionCompositionFailure(BaseModel):
    """Transport-neutral failure returned when no trajectory result is available."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-failure/v1"] = Field(
        default="taoryx.mission-composition-failure/v1",
        alias="schema",
        serialization_alias="schema",
    )
    category: FailureCategory
    phase: DiagnosticPhase
    operation: Literal["discover", "validate", "batch", "step"]
    retryable: bool
    request_id: str | None = None
    provider_id: str | None = None
    provider_version: str | None = None
    model_id: str | None = None
    configuration_fingerprint: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_failure(self) -> MissionCompositionFailure:
        if not any(item.severity == "error" for item in self.diagnostics):
            raise ValueError("a Mission Composition failure requires at least one error diagnostic")
        if self.retryable and not any(item.recoverability == "retryable" for item in self.diagnostics):
            raise ValueError("a retryable failure requires a retryable diagnostic")
        return self
        ####

    ####


class MissionCompositionExecutionError(RuntimeError):
    """Provider-authored structured exception caught by the common runner."""

    def __init__(self, diagnostic: MissionCompositionDiagnostic, *, category: FailureCategory = "execution_failed") -> None:
        if diagnostic.severity != "error":
            raise ValueError("MissionCompositionExecutionError requires an error diagnostic")
        self.diagnostic = diagnostic
        self.category = category
        super().__init__(f"{diagnostic.code}: {diagnostic.message}")
        ####

    ####


class MissionCompositionOutputSelection(BaseModel):
    """Provider-independent core-state and telemetry inclusion request."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mode: Literal["core", "selected", "all"] = "core"
    cadence_s: float | None = Field(default=None, gt=0.0)
    channels: tuple[str, ...] = ()
    telemetry_groups: tuple[str, ...] = ()
    include_events: bool = True
    include_segments: bool = True
    include_spawned_objects: bool = True
    maximum_samples_per_object: int | None = Field(default=None, gt=0)
    maximum_objects: int | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def validate_selection(self) -> MissionCompositionOutputSelection:
        if len(self.channels) != len(set(self.channels)):
            raise ValueError("output channel selection contains duplicates")
        if len(self.telemetry_groups) != len(set(self.telemetry_groups)):
            raise ValueError("output telemetry-group selection contains duplicates")
        if self.mode != "selected" and (self.channels or self.telemetry_groups):
            raise ValueError("explicit channels and telemetry groups require output mode 'selected'")
        return self
        ####

    ####


def resolve_output_selection(
    schema: TrajectoryOutputSchema,
    selection: MissionCompositionOutputSelection,
    *,
    fidelity: str,
    operation: Literal["batch", "step"],
    realization_id: str | None = None,
    mission_template_id: str | None = None,
) -> tuple[TrajectoryOutputChannelMetadata, ...]:
    """Resolve core plus requested telemetry without provider-specific rules."""

    def applicable(channel: TrajectoryOutputChannelMetadata) -> bool:
        return (
            (not channel.compatible_fidelities or fidelity in channel.compatible_fidelities)
            and (not channel.compatible_realizations or realization_id in channel.compatible_realizations)
            and (not channel.compatible_mission_templates or mission_template_id in channel.compatible_mission_templates)
            and operation in channel.operations
        )
        ####

    core = tuple(item for item in schema.core_channels if applicable(item))
    if not core:
        raise ValueError(f"output schema {schema.model_id!r} has no core channels for {fidelity!r}/{operation!r}")
    if selection.mode == "core":
        return core

    telemetry_by_id = {item.id: item for item in schema.telemetry_channels}
    if selection.mode == "all":
        requested_ids = {item.id for item in schema.telemetry_channels if applicable(item)}
    else:
        all_channel_ids = {item.id for item in schema.channels}
        unknown_channels = sorted(set(selection.channels) - all_channel_ids)
        if unknown_channels:
            raise ValueError(f"output channels are not advertised: {unknown_channels!r}")
        group_by_id = {item.id: item for item in schema.telemetry_groups}
        unknown_groups = sorted(set(selection.telemetry_groups) - set(group_by_id))
        if unknown_groups:
            raise ValueError(f"output telemetry groups are not advertised: {unknown_groups!r}")
        requested_ids = {item for item in selection.channels if item in telemetry_by_id}
        for group_id in selection.telemetry_groups:
            requested_ids.update(group_by_id[group_id].channel_ids)
    unavailable = sorted(channel_id for channel_id in requested_ids if not applicable(telemetry_by_id[channel_id]))
    if unavailable:
        raise ValueError(f"output telemetry channels are unavailable for {fidelity!r}/{operation!r}: {unavailable!r}")
    telemetry = tuple(item for item in schema.telemetry_channels if item.id in requested_ids)
    return (*core, *telemetry)
    ####


class MissionCompositionRunRequest(BaseModel):
    """Immutable batch handoff accepted by a common Mission Composition runner.

    Interactive execution is intentionally not represented as a stateless
    ``step`` request.  It uses the session lifecycle contract so state
    ownership, sequencing, reset, and close semantics remain explicit.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-run-request/v1"] = Field(
        default="taoryx.mission-composition-run-request/v1",
        alias="schema",
        serialization_alias="schema",
    )
    request_id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    operation: Literal["batch"] = "batch"
    prepared_configuration: PreparedTrajectoryConfiguration
    deployment_bindings: tuple[DeploymentBinding, ...] = ()
    output: MissionCompositionOutputSelection = Field(default_factory=MissionCompositionOutputSelection)

    @model_validator(mode="after")
    def validate_selected_deployment_bindings(self) -> MissionCompositionRunRequest:
        """Keep child selection explicit at the common request boundary."""

        validate_deployment_bindings(self.deployment_bindings)
        return self
        ####

    @property
    def model_id(self) -> str:
        return self.prepared_configuration.configuration.model_id
        ####

    ####


class TrajectoryChannelMetadata(BaseModel):
    """Semantic meaning and representation of one returned trajectory channel."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    channel_class: Literal["core_state", "telemetry"] = "core_state"
    telemetry_group: str | None = None
    quantity: str | None = None
    unit: str | None = None
    data_type: TrajectoryOutputDataType = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    frame: str | None = None
    sampling_semantics: TrajectorySamplingSemantics = "continuous_sample"
    interpolation: Literal["linear", "step", "periodic", "slerp", "event"] = "linear"
    description: str = ""

    @model_validator(mode="after")
    def validate_classification(self) -> TrajectoryChannelMetadata:
        if self.channel_class == "core_state" and self.telemetry_group is not None:
            raise ValueError(f"core trajectory channel {self.id!r} cannot name a telemetry group")
        if self.channel_class == "telemetry" and self.telemetry_group is None:
            raise ValueError(f"telemetry channel {self.id!r} requires a telemetry group")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"trajectory channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.unit is not None:
            raise ValueError(f"non-numeric trajectory channel {self.id!r} cannot advertise units")
        if self.interpolation == "event" and self.sampling_semantics != "event":
            raise ValueError(f"event trajectory channel {self.id!r} requires event sampling semantics")
        return self
        ####


class TrajectorySample(BaseModel):
    """One accepted truth sample for exactly one mission object."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    values: dict[str, Any]
    segment_instance_id: str | None = None

    @model_validator(mode="after")
    def validate_values(self) -> TrajectorySample:
        if not math.isfinite(self.time_s):
            raise ValueError("trajectory sample time must be finite")
        for channel, value in self.values.items():
            _validate_json_output_value(value, path=f"trajectory sample channel {channel!r}")
        return self

    ####

    ####


class TrajectoryStateSnapshot(BaseModel):
    """Entity state at one accepted boundary, using the entity's channel metadata."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    values: dict[str, Any] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_snapshot(self) -> TrajectoryStateSnapshot:
        if not math.isfinite(self.time_s):
            raise ValueError("trajectory state snapshot time must be finite")
        for channel, value in self.values.items():
            _validate_json_output_value(value, path=f"trajectory state snapshot channel {channel!r}")
        return self
        ####

    ####


class TrajectoryEvent(BaseModel):
    """Accepted-boundary event shared by trajectory and lineage consumers."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    time_s: float = Field(ge=0.0)
    category: Literal[
        "segment",
        "burnout",
        "release",
        "separation",
        "deployment",
        "impact",
        "termination",
        "resource",
        "custom",
    ]
    kind: str = Field(min_length=1)
    object_id: str = Field(min_length=1)
    parent_object_id: str | None = None
    deployment_id: str | None = None
    segment_instance_id: str | None = None
    detail: str = ""
    data: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_time(self) -> TrajectoryEvent:
        if not math.isfinite(self.time_s):
            raise ValueError("trajectory event time must be finite")
        return self

    ####

    ####


class TrajectoryEntityRelationship(BaseModel):
    """First-class accepted-boundary relationship between a parent and spawned child."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    kind: Literal["release", "separation", "deployment"]
    parent_object_id: str = Field(min_length=1)
    child_object_id: str = Field(min_length=1)
    deployment_id: str = Field(min_length=1)
    event_id: str = Field(min_length=1)
    created_at_s: float = Field(ge=0.0)
    state_transfer: str = Field(min_length=1)
    initial_state: TrajectoryStateSnapshot
    provenance: str = ""

    @model_validator(mode="after")
    def validate_relationship(self) -> TrajectoryEntityRelationship:
        if self.parent_object_id == self.child_object_id:
            raise ValueError("an entity relationship cannot make an object its own child")
        if not math.isfinite(self.created_at_s):
            raise ValueError("entity relationship creation time must be finite")
        if abs(self.initial_state.time_s - self.created_at_s) > 1.0e-9:
            raise ValueError("entity relationship initial-state time must equal created_at_s")
        return self
        ####

    ####


class TrajectorySegmentResult(BaseModel):
    """Observed segment span for one object, including linked event IDs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    instance_id: str = Field(min_length=1)
    object_id: str = Field(min_length=1)
    start_time_s: float = Field(ge=0.0)
    end_time_s: float = Field(ge=0.0)
    status: TrajectorySegmentStatus
    event_ids: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_span(self) -> TrajectorySegmentResult:
        if not math.isfinite(self.start_time_s) or not math.isfinite(self.end_time_s):
            raise ValueError("segment result times must be finite")
        if self.end_time_s < self.start_time_s:
            raise ValueError(f"segment result {self.instance_id!r} ends before it starts")
        return self
        ####

    ####


class TrajectoryObject(BaseModel):
    """One primary or emitted model and its independently sampled history."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    object_id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    realization_id: str = Field(min_length=1)
    name: str = Field(min_length=1)
    role: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    status: TrajectoryObjectStatus
    parent_object_id: str | None = None
    deployment_id: str | None = None
    spawn_event_id: str | None = None
    death_event_id: str | None = None
    active_from_s: float = Field(ge=0.0)
    active_to_s: float | None = Field(default=None, ge=0.0)
    terminal_disposition: str | None = None
    channels: tuple[TrajectoryChannelMetadata, ...] = Field(min_length=1)
    samples: tuple[TrajectorySample, ...] = Field(min_length=1)
    segments: tuple[TrajectorySegmentResult, ...] = ()
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_history(self) -> TrajectoryObject:
        if self.active_to_s is not None and self.active_to_s < self.active_from_s:
            raise ValueError(f"trajectory object {self.object_id!r} has an inverted active interval")
        if self.status == "active" and self.active_to_s is not None:
            raise ValueError(f"active trajectory object {self.object_id!r} cannot declare active_to_s")
        if self.status != "active" and self.active_to_s is None:
            raise ValueError(f"terminal trajectory object {self.object_id!r} requires active_to_s")
        channel_ids = tuple(item.id for item in self.channels)
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError(f"trajectory object {self.object_id!r} has duplicate channel IDs")
        if not any(item.channel_class == "core_state" for item in self.channels):
            raise ValueError(f"trajectory object {self.object_id!r} requires at least one core-state channel")
        expected_channels = set(channel_ids)
        channel_by_id = {item.id: item for item in self.channels}
        times = tuple(item.time_s for item in self.samples)
        if any(later < earlier for earlier, later in zip(times, times[1:], strict=False)):
            raise ValueError(f"trajectory object {self.object_id!r} samples are not monotonic")
        if abs(times[0] - self.active_from_s) > 1.0e-9:
            raise ValueError(f"trajectory object {self.object_id!r} first sample must equal active_from_s")
        if self.active_to_s is not None and times[-1] > self.active_to_s + 1.0e-9:
            raise ValueError(f"trajectory object {self.object_id!r} samples extend beyond active_to_s")
        if self.active_to_s is not None and abs(times[-1] - self.active_to_s) > 1.0e-9:
            raise ValueError(f"trajectory object {self.object_id!r} last sample must equal active_to_s")
        for index, sample in enumerate(self.samples):
            if set(sample.values) != expected_channels:
                raise ValueError(f"trajectory object {self.object_id!r} sample {index} channel set does not match channel metadata")
            for channel_id, value in sample.values.items():
                _validate_typed_output_value(
                    channel_by_id[channel_id],
                    value,
                    path=f"trajectory object {self.object_id!r} sample {index} channel {channel_id!r}",
                )
        segment_ids = tuple(item.instance_id for item in self.segments)
        if len(segment_ids) != len(set(segment_ids)):
            raise ValueError(f"trajectory object {self.object_id!r} has duplicate segment instance IDs")
        for segment in self.segments:
            if segment.object_id != self.object_id:
                raise ValueError(f"trajectory object {self.object_id!r} contains a segment owned by another object")
            if segment.start_time_s < self.active_from_s - 1.0e-9:
                raise ValueError(f"segment {segment.instance_id!r} starts before its object becomes active")
            if self.active_to_s is not None and segment.end_time_s > self.active_to_s + 1.0e-9:
                raise ValueError(f"segment {segment.instance_id!r} ends after its object becomes inactive")
        return self
        ####

    ####


class MissionCompositionTrajectoryResult(BaseModel):
    """Standard provider-independent, multi-object Mission Composition result."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-trajectory/v1"] = Field(
        default="taoryx.mission-composition-trajectory/v1",
        alias="schema",
        serialization_alias="schema",
    )
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    request_id: str = Field(min_length=1)
    configuration_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    primary_model_id: str = Field(min_length=1)
    primary_object_id: str = Field(min_length=1)
    status: TrajectoryOutcome
    objects: tuple[TrajectoryObject, ...] = Field(min_length=1)
    events: tuple[TrajectoryEvent, ...] = ()
    relationships: tuple[TrajectoryEntityRelationship, ...] = ()
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_result(self) -> MissionCompositionTrajectoryResult:
        object_ids = tuple(item.object_id for item in self.objects)
        if len(object_ids) != len(set(object_ids)):
            raise ValueError("trajectory result object IDs must be unique")
        object_by_id = {item.object_id: item for item in self.objects}
        primary = object_by_id.get(self.primary_object_id)
        if primary is None:
            raise ValueError("primary_object_id does not identify a returned object")
        if primary.model_id != self.primary_model_id:
            raise ValueError("primary_model_id does not match the primary trajectory object")
        if primary.parent_object_id is not None or primary.deployment_id is not None or primary.spawn_event_id is not None:
            raise ValueError("the primary trajectory object cannot have a parent, deployment, or spawn event")
        event_ids = tuple(item.id for item in self.events)
        if len(event_ids) != len(set(event_ids)):
            raise ValueError("trajectory result event IDs must be unique")
        event_by_id = {item.id: item for item in self.events}
        event_times = tuple(item.time_s for item in self.events)
        if any(later < earlier for earlier, later in zip(event_times, event_times[1:], strict=False)):
            raise ValueError("trajectory result events must be ordered by accepted time")
        for event in self.events:
            if event.object_id not in object_by_id:
                raise ValueError(f"trajectory event {event.id!r} references unknown object {event.object_id!r}")
            if event.parent_object_id is not None and event.parent_object_id not in object_by_id:
                raise ValueError(f"trajectory event {event.id!r} references unknown parent {event.parent_object_id!r}")
        relationship_ids = tuple(item.id for item in self.relationships)
        if len(relationship_ids) != len(set(relationship_ids)):
            raise ValueError("trajectory result relationship IDs must be unique")
        relationship_by_child: dict[str, TrajectoryEntityRelationship] = {}
        for relationship in self.relationships:
            if relationship.parent_object_id not in object_by_id or relationship.child_object_id not in object_by_id:
                raise ValueError(f"trajectory relationship {relationship.id!r} references an unknown object")
            if relationship.child_object_id in relationship_by_child:
                raise ValueError(f"trajectory child {relationship.child_object_id!r} has multiple spawn relationships")
            relationship_by_child[relationship.child_object_id] = relationship
            relationship_event = event_by_id.get(relationship.event_id)
            if relationship_event is None:
                raise ValueError(f"trajectory relationship {relationship.id!r} references an unknown event")
            if relationship_event.category != relationship.kind:
                raise ValueError(f"trajectory relationship {relationship.id!r} kind does not match its event")
            if relationship_event.object_id != relationship.child_object_id or relationship_event.parent_object_id != relationship.parent_object_id:
                raise ValueError(f"trajectory relationship {relationship.id!r} event has inconsistent lineage")
            if relationship_event.deployment_id != relationship.deployment_id:
                raise ValueError(f"trajectory relationship {relationship.id!r} event has inconsistent deployment identity")
            if abs(relationship_event.time_s - relationship.created_at_s) > 1.0e-9:
                raise ValueError(f"trajectory relationship {relationship.id!r} event time is inconsistent")
        for item in self.objects:
            if item.parent_object_id is not None and item.parent_object_id not in object_by_id:
                raise ValueError(f"trajectory object {item.object_id!r} references unknown parent {item.parent_object_id!r}")
            if item.parent_object_id is None and (item.deployment_id is not None or item.spawn_event_id is not None):
                raise ValueError(f"root trajectory object {item.object_id!r} cannot name a deployment or spawn event")
            if item.parent_object_id is not None and item.deployment_id is None:
                raise ValueError(f"child trajectory object {item.object_id!r} requires a deployment_id")
            if item.spawn_event_id is not None:
                spawn_event = event_by_id.get(item.spawn_event_id)
                if spawn_event is None:
                    raise ValueError(f"trajectory object {item.object_id!r} references unknown spawn event")
                if spawn_event.object_id != item.object_id or spawn_event.parent_object_id != item.parent_object_id:
                    raise ValueError(f"trajectory object {item.object_id!r} spawn event has inconsistent lineage")
                if spawn_event.deployment_id != item.deployment_id:
                    raise ValueError(f"trajectory object {item.object_id!r} spawn event has inconsistent deployment identity")
                if spawn_event.category not in {"release", "separation", "deployment"}:
                    raise ValueError(f"trajectory object {item.object_id!r} spawn event has a non-deployment category")
                if abs(spawn_event.time_s - item.active_from_s) > 1.0e-9:
                    raise ValueError(f"trajectory object {item.object_id!r} spawn event time does not match active_from_s")
            elif item.parent_object_id is not None:
                raise ValueError(f"child trajectory object {item.object_id!r} requires a spawn event")
            spawn_relationship = relationship_by_child.get(item.object_id)
            if item.parent_object_id is None and spawn_relationship is not None:
                raise ValueError(f"root trajectory object {item.object_id!r} cannot have a spawn relationship")
            if item.parent_object_id is not None:
                if spawn_relationship is None:
                    raise ValueError(f"child trajectory object {item.object_id!r} requires a spawn relationship")
                if (
                    spawn_relationship.parent_object_id != item.parent_object_id
                    or spawn_relationship.deployment_id != item.deployment_id
                    or spawn_relationship.event_id != item.spawn_event_id
                ):
                    raise ValueError(f"child trajectory object {item.object_id!r} has inconsistent relationship metadata")
                if abs(spawn_relationship.created_at_s - item.active_from_s) > 1.0e-9:
                    raise ValueError(f"child trajectory object {item.object_id!r} relationship time is inconsistent")
                first_sample = item.samples[0]
                if spawn_relationship.initial_state.values != first_sample.values:
                    raise ValueError(f"child trajectory object {item.object_id!r} initial state does not match its first sample")
            if item.parent_object_id is not None:
                parent = object_by_id[item.parent_object_id]
                if item.active_from_s < parent.active_from_s - 1.0e-9:
                    raise ValueError(f"child trajectory object {item.object_id!r} becomes active before its parent")
                if parent.active_to_s is not None and item.active_from_s > parent.active_to_s + 1.0e-9:
                    raise ValueError(f"child trajectory object {item.object_id!r} spawns after its parent becomes inactive")
            if item.death_event_id is not None:
                death_event = event_by_id.get(item.death_event_id)
                if death_event is None or death_event.object_id != item.object_id:
                    raise ValueError(f"trajectory object {item.object_id!r} death event is missing or inconsistent")
                if item.active_to_s is None or abs(death_event.time_s - item.active_to_s) > 1.0e-9:
                    raise ValueError(f"trajectory object {item.object_id!r} death event time does not match active_to_s")
            for segment in item.segments:
                unknown_events = sorted(set(segment.event_ids) - set(event_by_id))
                if unknown_events:
                    raise ValueError(f"segment {segment.instance_id!r} references unknown events {unknown_events!r}")
        _validate_acyclic_lineage(object_by_id)
        has_error = any(item.severity == "error" for item in self.diagnostics)
        active_objects = tuple(item.object_id for item in self.objects if item.status == "active")
        if self.status == "in_progress" and not active_objects:
            raise ValueError("an in-progress trajectory result requires at least one active object")
        if self.status != "in_progress" and active_objects:
            raise ValueError(f"a terminal trajectory result cannot contain active objects: {active_objects!r}")
        if self.status in {"in_progress", "completed", "terminated"} and has_error:
            raise ValueError("successful or in-progress result cannot contain error diagnostics")
        if self.status in {"partial", "failed"} and not has_error:
            raise ValueError("partial or failed result requires an error diagnostic")
        return self
        ####

    def to_dict(self) -> dict[str, object]:
        return self.model_dump(mode="json", by_alias=True)
        ####

    ####


class MissionCompositionTrajectoryResponse(BaseModel):
    """Common-runner response containing accepted trajectory samples."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["trajectory"] = "trajectory"
    result: MissionCompositionTrajectoryResult


class MissionCompositionFailureResponse(BaseModel):
    """Rejected or failed common-runner response."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    kind: Literal["failure"] = "failure"
    failure: MissionCompositionFailure


MissionCompositionRunResponse = Annotated[
    MissionCompositionTrajectoryResponse | MissionCompositionFailureResponse,
    Field(discriminator="kind"),
]
_RESPONSE_ADAPTER: TypeAdapter[MissionCompositionRunResponse] = TypeAdapter(MissionCompositionRunResponse)
MissionCompositionExecutor = Callable[[MissionCompositionRunRequest], MissionCompositionTrajectoryResult]


class MissionCompositionRunnerRegistry:
    """Dispatch common run requests and contain provider exceptions."""

    def __init__(self, executors: Mapping[tuple[str, str], MissionCompositionExecutor] | None = None) -> None:
        self._executors = dict(executors or {})
        ####

    def register(self, provider_id: str, model_id: str, executor: MissionCompositionExecutor) -> None:
        key = (provider_id, model_id)
        if key in self._executors:
            raise ValueError(f"Mission Composition executor {key!r} is already registered")
        self._executors[key] = executor
        ####

    def has_executor(self, provider_id: str, model_id: str) -> bool:
        """Return whether the exact provider/model dispatch key is registered."""

        return (provider_id, model_id) in self._executors
        ####

    def registrations(self) -> tuple[tuple[str, str], ...]:
        """Return deterministic executor keys for diagnostics and host introspection."""

        return tuple(sorted(self._executors))
        ####

    def run(self, request: MissionCompositionRunRequest) -> MissionCompositionRunResponse:
        key = (request.provider_id, request.model_id)
        executor = self._executors.get(key)
        if executor is None:
            return _failure_response(
                request,
                MissionCompositionDiagnostic(
                    severity="error",
                    code="executor-not-registered",
                    message="No executor is registered for the requested provider and model.",
                    phase="preflight",
                    recoverability="correctable",
                    path="/provider_id",
                    hint="Select an advertised runnable operation or install the model's execution binding.",
                    provider_id=request.provider_id,
                    model_id=request.model_id,
                ),
                category="unsupported",
            )
        try:
            result = executor(request)
            _validate_executor_result(request, result)
        except Exception as error:  # The plug-in boundary must not leak provider-specific exception types.
            diagnostic, category = diagnostic_from_exception(
                error,
                phase="execution",
                provider_id=request.provider_id,
                model_id=request.model_id,
            )
            return _failure_response(request, diagnostic, category=category)
        return MissionCompositionTrajectoryResponse(result=result)
        ####

    ####


@runtime_checkable
class RunnableMissionCompositionProvider(ConfigurableTrajectoryProvider, Protocol):
    """Discovery provider that also publishes the common execution surface.

    Configuration discovery remains useful without a local executor, so it is
    intentionally modeled separately from this optional capability.  Hosts
    that dispatch ``model run`` can now test an explicit contract rather than
    reaching into a provider for an untyped attribute.
    """

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Build the provider-owned registry of common Mission Composition executors."""

        ...

    ####


class ProviderModelAdvertisementConformance(BaseModel):
    """Conformance result for one advertised model and its full schema."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    configuration_schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    output_schema_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    status: Literal["pass", "fail"]
    common_runner_operations: tuple[Literal["batch", "step"], ...] = ()
    adapter_required_operations: tuple[Literal["batch", "step"], ...] = ()
    deployment_ids: tuple[str, ...] = ()
    deployment_adapter_required_ids: tuple[str, ...] = ()
    configuration_node_kinds: tuple[str, ...] = ()
    configuration_value_types: tuple[str, ...] = ()
    model_property_count: int = Field(default=0, ge=0)
    reference_frame_count: int = Field(default=0, ge=0)
    control_statuses: tuple[TrajectoryControlStatus, ...] = ()
    control_channel_count: int = Field(default=0, ge=0)
    action_control_channel_count: int = Field(default=0, ge=0)
    effector_control_channel_count: int = Field(default=0, ge=0)
    active_control_channel_count: int = Field(default=0, ge=0)
    native_bound_control_channel_count: int = Field(default=0, ge=0)
    control_authority_count: int = Field(default=0, ge=0)
    control_intent_count: int = Field(default=0, ge=0)
    output_channel_count: int = Field(default=0, ge=0)
    core_output_channel_count: int = Field(default=0, ge=0)
    telemetry_channel_count: int = Field(default=0, ge=0)
    telemetry_group_count: int = Field(default=0, ge=0)
    registered_batch_tuple_count: int = Field(default=0, ge=0)
    registered_step_tuple_count: int = Field(default=0, ge=0)
    blocked_execution_tuple_count: int = Field(default=0, ge=0)
    registered_executor_ids: tuple[str, ...] = ()
    supports_dynamic_spawning: bool = False
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()


class ProviderAdvertisementConformanceReport(BaseModel):
    """Machine-readable proof that one provider publication is internally usable."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-advertisement-audit/v1"] = Field(
        default="taoryx.mission-composition-advertisement-audit/v1",
        alias="schema",
        serialization_alias="schema",
    )
    provider_id: str
    provider_version: str
    status: Literal["pass", "fail"]
    advertised_model_count: int = Field(ge=0)
    validated_model_count: int = Field(ge=0)
    runner_checked: bool = False
    models: tuple[ProviderModelAdvertisementConformance, ...] = ()
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()


def audit_provider_advertisement(
    provider: ConfigurableTrajectoryProvider,
    runner: MissionCompositionRunnerRegistry | None = None,
) -> ProviderAdvertisementConformanceReport:
    """Load and JSON-round-trip every advertised schema and cross-reference."""

    diagnostics: list[MissionCompositionDiagnostic] = []
    metadata = provider.metadata
    try:
        type(metadata).model_validate_json(metadata.model_dump_json(by_alias=True))
    except Exception as error:
        diagnostics.append(
            _audit_diagnostic(
                "provider-metadata-not-serializable",
                f"Provider metadata could not round-trip through JSON ({type(error).__name__}).",
                metadata.id,
            )
        )
    try:
        models = provider.list_models()
    except Exception as error:
        diagnostic, _ = diagnostic_from_exception(error, phase="discovery", provider_id=metadata.id)
        diagnostics.append(diagnostic)
        models = ()
    model_ids = tuple(item.id for item in models)
    if len(model_ids) != len(set(model_ids)):
        diagnostics.append(_audit_diagnostic("duplicate-model-id", "Provider advertises duplicate model IDs.", metadata.id))
    if metadata.model_count != len(models):
        diagnostics.append(
            _audit_diagnostic(
                "model-count-mismatch",
                f"Provider metadata declares {metadata.model_count} models but list_models returned {len(models)}.",
                metadata.id,
            )
        )
    model_reports: list[ProviderModelAdvertisementConformance] = []
    for model in models:
        model_diagnostics: list[MissionCompositionDiagnostic] = []
        node_kinds: tuple[str, ...] = ()
        value_types: tuple[str, ...] = ()
        execution_operations = tuple(item for mission in model.mission_templates for item in mission.operations if item.operation in _RUN_OPERATIONS)
        registered_execution_operations = tuple(item for item in execution_operations if item.common_runner_status == "registered")
        registered_batch_tuples = tuple(
            (mission.id, item)
            for mission in model.mission_templates
            for item in mission.operations
            if item.operation == "batch" and item.common_runner_status == "registered"
        )
        for item in execution_operations:
            if item.status == "available" and item.common_runner_status != "registered":
                model_diagnostics.append(
                    _audit_diagnostic(
                        "available-operation-not-registered",
                        (f"Available {item.operation} tuple {item.fidelity!r}/{item.realization_id!r} does not resolve through the common interface."),
                        metadata.id,
                        model_id=model.id,
                    )
                )
            if item.common_runner_status == "registered" and item.executor_id is None:
                model_diagnostics.append(
                    _audit_diagnostic(
                        "registered-operation-executor-id-missing",
                        f"Registered {item.operation} tuple {item.fidelity!r}/{item.realization_id!r} has no executor ID.",
                        metadata.id,
                        model_id=model.id,
                    )
                )
        if model.mission_templates:
            exact_common_operations = {item.operation for item in registered_execution_operations}
            if set(model.common_runner_operations) != exact_common_operations:
                model_diagnostics.append(
                    _audit_diagnostic(
                        "model-operation-summary-mismatch",
                        (
                            f"Model common_runner_operations={sorted(model.common_runner_operations)!r} does not equal "
                            f"its exact registered mission operations={sorted(exact_common_operations)!r}."
                        ),
                        metadata.id,
                        model_id=model.id,
                    )
                )
            for realization in model.realizations:
                expected = {item for item in realization.operations if item in _RUN_OPERATIONS}
                if not expected:
                    continue
                witnessed = {item.operation for item in registered_execution_operations if item.realization_id == realization.id}
                missing_realization_operations = sorted(expected - witnessed)
                if missing_realization_operations:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "realization-operation-tuple-missing",
                            (
                                f"Available realization {realization.id!r} advertises operations "
                                f"{missing_realization_operations!r} without exact registered mission tuples."
                            ),
                            metadata.id,
                            model_id=model.id,
                        )
                    )
        for realization in model.realizations:
            active_channels = tuple(item for item in realization.controls.channels if item.operations)
            active_authorities = tuple(item for item in realization.controls.authorities if item.operations)
            if realization.controls.status == "available" and not active_authorities:
                model_diagnostics.append(
                    _audit_diagnostic(
                        "control-authority-missing",
                        f"Externally controlled realization {realization.id!r} has no active authority profile.",
                        metadata.id,
                        model_id=model.id,
                    )
                )
            if "step" in realization.operations:
                step_actions = tuple(item for item in active_channels if item.channel_kind == "action" and "step" in item.operations)
                source_program_owns_step = (
                    realization.controls.status == "internally_generated"
                    and any(
                        authority.command_owner == "source_program" and "step" in authority.operations
                        for authority in active_authorities
                    )
                )
                if not step_actions and not source_program_owns_step:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "interactive-control-schema-missing",
                            f"Interactive realization {realization.id!r} has no advertised step action channels.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
                missing_native_ids = sorted(item.id for item in step_actions if item.native_binding is None)
                if missing_native_ids:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "interactive-control-native-binding-missing",
                            (f"Interactive realization {realization.id!r} has action channels without native IDs {missing_native_ids!r}."),
                            metadata.id,
                            model_id=model.id,
                        )
                    )
        advertised_output_operations = {operation for channel in model.output_channels for operation in channel.operations}
        unavailable_output_operations = sorted(advertised_output_operations - set(model.common_runner_operations))
        if unavailable_output_operations:
            model_diagnostics.append(
                _audit_diagnostic(
                    "output-operation-not-executable",
                    f"Output channels advertise unavailable operations {unavailable_output_operations!r}.",
                    metadata.id,
                    model_id=model.id,
                )
            )
        try:
            type(model).model_validate_json(model.model_dump_json())
        except Exception as error:
            model_diagnostics.append(
                _audit_diagnostic(
                    "model-metadata-not-serializable",
                    f"Model metadata could not round-trip through JSON ({type(error).__name__}).",
                    metadata.id,
                    model_id=model.id,
                )
            )
        if not model.output_channels:
            model_diagnostics.append(
                _audit_diagnostic(
                    "output-metadata-missing",
                    "Model does not advertise any potential or guaranteed output channels.",
                    metadata.id,
                    model_id=model.id,
                )
            )
        try:
            output_schema = provider.get_model_output_schema(model.id)
            round_tripped_output = type(output_schema).model_validate_json(output_schema.model_dump_json(by_alias=True))
            output_checks = (
                (output_schema.model_id == model.id, "output-schema-model-mismatch", "Output schema model_id does not match model metadata."),
                (output_schema.model_version == model.version, "output-schema-version-mismatch", "Output schema model_version does not match model metadata."),
                (
                    output_schema.schema_id == model.output_schema_id,
                    "output-schema-id-mismatch",
                    "Output schema ID does not match model metadata.",
                ),
                (
                    output_schema.fingerprint == model.output_schema_fingerprint,
                    "output-schema-fingerprint-mismatch",
                    "Published output schema identity does not match model metadata.",
                ),
                (
                    round_tripped_output.fingerprint == output_schema.fingerprint,
                    "output-schema-roundtrip-mismatch",
                    "Output schema identity changed during a JSON round trip.",
                ),
            )
            for passed, code, message in output_checks:
                if not passed:
                    model_diagnostics.append(_audit_diagnostic(code, message, metadata.id, model_id=model.id))
            for mission_id, operation in registered_batch_tuples:
                try:
                    resolve_output_selection(
                        output_schema,
                        MissionCompositionOutputSelection(mode="core"),
                        fidelity=operation.fidelity,
                        operation="batch",
                        realization_id=operation.realization_id,
                        mission_template_id=mission_id,
                    )
                except ValueError as error:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "registered-batch-output-core-missing",
                            (
                                f"Registered batch tuple {mission_id!r}/{operation.fidelity!r}/"
                                f"{operation.realization_id!r} has no selectable core output ({error})."
                            ),
                            metadata.id,
                            model_id=model.id,
                        )
                    )
        except Exception as error:
            diagnostic, _ = diagnostic_from_exception(
                error,
                phase="discovery",
                provider_id=metadata.id,
                model_id=model.id,
            )
            model_diagnostics.append(diagnostic)
        try:
            schema = provider.get_model_schema(model.id)
        except Exception as error:
            diagnostic, _ = diagnostic_from_exception(
                error,
                phase="discovery",
                provider_id=metadata.id,
                model_id=model.id,
            )
            model_diagnostics.append(diagnostic)
        else:
            node_kinds, value_types, parameter_frames = _configuration_contract_coverage(schema.root)
            checks = (
                (schema.model_id == model.id, "schema-model-mismatch", "Schema model_id does not match model metadata."),
                (
                    schema.model_version == model.version,
                    "schema-version-mismatch",
                    "Schema model_version does not match model metadata.",
                ),
                (
                    schema.schema_id == model.configuration_schema_id,
                    "schema-id-mismatch",
                    "Configuration schema ID does not match model metadata.",
                ),
                (
                    schema.fingerprint == model.configuration_schema_fingerprint,
                    "schema-fingerprint-mismatch",
                    "Configuration schema fingerprint does not match model metadata.",
                ),
            )
            for passed, code, message in checks:
                if not passed:
                    model_diagnostics.append(_audit_diagnostic(code, message, metadata.id, model_id=model.id))
            unknown_parameter_frames = sorted(set(parameter_frames) - {item.id for item in model.reference_frames})
            if unknown_parameter_frames:
                model_diagnostics.append(
                    _audit_diagnostic(
                        "unknown-configuration-frame",
                        f"Configuration parameters reference unadvertised frames {unknown_parameter_frames!r}.",
                        metadata.id,
                        model_id=model.id,
                    )
                )
            try:
                round_tripped = type(schema).model_validate_json(schema.model_dump_json(by_alias=True))
                if round_tripped.fingerprint != schema.fingerprint:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "schema-roundtrip-mismatch",
                            "Configuration schema identity changed during a JSON round trip.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
            except Exception as error:
                model_diagnostics.append(
                    _audit_diagnostic(
                        "schema-not-serializable",
                        f"Configuration schema could not round-trip through JSON ({type(error).__name__}).",
                        metadata.id,
                        model_id=model.id,
                    )
                )
            fidelity_by_id = {item.id: item for item in model.fidelities}
            for fidelity in schema.supported_fidelities:
                record = fidelity_by_id.get(fidelity)
                if record is None or not record.declared:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "undeclared-schema-fidelity",
                            f"Configuration schema supports undeclared fidelity {fidelity!r}.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
            segment_ids = {segment for mission in model.mission_templates for segment in mission.advertised_segment_ids}
            for deployment in model.deployments:
                unknown_segments = sorted(set(deployment.trigger_segment_ids) - segment_ids)
                if unknown_segments:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "unknown-deployment-trigger-segment",
                            f"Deployment {deployment.id!r} references unadvertised trigger segments {unknown_segments!r}.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
                unavailable_operations = sorted({str(item) for item in deployment.operations} - {str(item) for item in model.operations})
                if unavailable_operations:
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "deployment-operation-mismatch",
                            f"Deployment {deployment.id!r} requires unavailable model operations {unavailable_operations!r}.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
                if deployment.status == "available" and (
                    deployment.common_runner_status not in {"registered", "adapter_required"} or deployment.executor_id is None
                ):
                    model_diagnostics.append(
                        _audit_diagnostic(
                            "available-deployment-not-registered",
                            f"Available deployment {deployment.id!r} has no registered executor or explicit adapter binding.",
                            metadata.id,
                            model_id=model.id,
                        )
                    )
            if runner is not None and model.common_runner_operations and not runner.has_executor(metadata.id, model.id):
                model_diagnostics.append(
                    _audit_diagnostic(
                        "common-runner-executor-missing",
                        "Model advertises common-runner operations but its provider/model executor is not registered.",
                        metadata.id,
                        model_id=model.id,
                    )
                )
        control_channels = tuple(channel for realization in model.realizations for channel in realization.controls.channels)
        model_reports.append(
            ProviderModelAdvertisementConformance(
                model_id=model.id,
                model_version=model.version,
                configuration_schema_fingerprint=model.configuration_schema_fingerprint,
                output_schema_fingerprint=model.output_schema_fingerprint,
                status="fail" if model_diagnostics else "pass",
                common_runner_operations=model.common_runner_operations,
                adapter_required_operations=tuple(
                    operation
                    for operation in _RUN_OPERATIONS
                    if any(
                        item.operation == operation and item.common_runner_status == "adapter_required"
                        for mission in model.mission_templates
                        for item in mission.operations
                    )
                ),
                deployment_ids=tuple(item.id for item in model.deployments),
                deployment_adapter_required_ids=tuple(item.id for item in model.deployments if item.common_runner_status == "adapter_required"),
                configuration_node_kinds=node_kinds,
                configuration_value_types=value_types,
                model_property_count=len(model.presentation.properties),
                reference_frame_count=len(model.reference_frames),
                control_statuses=tuple(dict.fromkeys(item.controls.status for item in model.realizations)),
                control_channel_count=len(control_channels),
                action_control_channel_count=sum(item.channel_kind == "action" for item in control_channels),
                effector_control_channel_count=sum(item.channel_kind == "effector" for item in control_channels),
                active_control_channel_count=sum(bool(item.operations) for item in control_channels),
                native_bound_control_channel_count=sum(item.native_binding is not None for item in control_channels),
                control_authority_count=sum(len(item.controls.authorities) for item in model.realizations),
                control_intent_count=sum(len(item.controls.intents) for item in model.realizations),
                output_channel_count=len(model.output_channels),
                core_output_channel_count=len(model.output_schema.core_channels),
                telemetry_channel_count=len(model.output_schema.telemetry_channels),
                telemetry_group_count=len(model.output_schema.telemetry_groups),
                registered_batch_tuple_count=sum(item.operation == "batch" for item in registered_execution_operations),
                registered_step_tuple_count=sum(item.operation == "step" for item in registered_execution_operations),
                blocked_execution_tuple_count=sum(item.status == "blocked" for item in execution_operations),
                registered_executor_ids=tuple(sorted({item.executor_id for item in registered_execution_operations if item.executor_id is not None})),
                supports_dynamic_spawning=model.output_schema.entity_output.supports_dynamic_spawning,
                diagnostics=tuple(model_diagnostics),
            )
        )
        diagnostics.extend(model_diagnostics)
    validated = sum(item.status == "pass" for item in model_reports)
    return ProviderAdvertisementConformanceReport(
        provider_id=metadata.id,
        provider_version=metadata.version,
        status="fail" if any(item.severity == "error" for item in diagnostics) else "pass",
        advertised_model_count=len(models),
        validated_model_count=validated,
        runner_checked=runner is not None,
        models=tuple(model_reports),
        diagnostics=tuple(diagnostics),
    )
    ####


def _configuration_contract_coverage(
    root: ConfigurationNode,
) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]:
    """Collect deterministic node, value-type, and frame coverage metadata."""

    node_kinds: set[str] = set()
    value_types: set[str] = set()
    frames: set[str] = set()

    def visit(node: ConfigurationNode) -> None:
        node_kinds.add(node.kind)
        if isinstance(node, ConfigurationParameterSchema):
            value_types.add(node.value_type)
            if node.frame is not None:
                frames.add(node.frame)
        elif isinstance(node, ConfigurationGroupSchema):
            for child in node.children:
                visit(child)
        elif isinstance(node, ConfigurationChoiceSchema):
            for variant in node.variants:
                visit(variant.node)
        elif isinstance(node, ConfigurationSequenceSchema):
            visit(node.item)
        elif isinstance(node, ConfigurationOptionalSchema):
            visit(node.item)
        ####

    visit(root)
    return tuple(sorted(node_kinds)), tuple(sorted(value_types)), tuple(sorted(frames))
    ####


def diagnostic_from_exception(
    error: Exception,
    *,
    phase: DiagnosticPhase,
    provider_id: str | None = None,
    model_id: str | None = None,
) -> tuple[MissionCompositionDiagnostic, FailureCategory]:
    """Normalize known and unknown provider exceptions without leaking types."""

    if isinstance(error, MissionCompositionExecutionError):
        diagnostic = error.diagnostic.model_copy(
            update={
                "provider_id": error.diagnostic.provider_id or provider_id,
                "model_id": error.diagnostic.model_id or model_id,
            }
        )
        return diagnostic, error.category
    if isinstance(error, ConfigurationContractError):
        return (
            MissionCompositionDiagnostic(
                severity="error",
                code=_normalize_code(error.code),
                message=_exception_message(error),
                phase="configuration",
                recoverability="correctable",
                path=_json_pointer(error.path),
                hint="Refresh the advertised schema and correct the configuration at the reported path.",
                provider_id=provider_id,
                model_id=model_id,
            ),
            "invalid_request",
        )
    if isinstance(error, ValidationError):
        return (
            MissionCompositionDiagnostic(
                severity="error",
                code="invalid-request",
                message="The request does not satisfy the common Mission Composition contract.",
                phase="configuration",
                recoverability="correctable",
                hint="Correct the reported fields and resubmit the request.",
                provider_id=provider_id,
                model_id=model_id,
                details={"errors": _validation_error_details(error)},
            ),
            "invalid_request",
        )
    duck_code = getattr(error, "code", None)
    if isinstance(duck_code, str):
        code = _normalize_code(duck_code)
        path = getattr(error, "path", None) or getattr(error, "field", None)
        category: FailureCategory = "unsupported" if code.startswith(("unknown-", "unsupported-", "missing-executor")) else "invalid_request"
        return (
            MissionCompositionDiagnostic(
                severity="error",
                code=code,
                message=_exception_message(error),
                phase="preflight" if category == "unsupported" else "configuration",
                recoverability="correctable",
                path=_json_pointer(path) if isinstance(path, str) else None,
                hint="Review the provider advertisement and correct the request before retrying.",
                provider_id=provider_id,
                model_id=model_id,
            ),
            category,
        )
    return (
        MissionCompositionDiagnostic(
            severity="error",
            code="provider-internal-error",
            message="The provider failed without a structured public diagnostic.",
            phase=phase,
            recoverability="fatal",
            hint="Inspect provider logs using the request ID; do not retry unchanged automatically.",
            provider_id=provider_id,
            model_id=model_id,
            details={"exception_type": type(error).__name__},
        ),
        "provider_error",
    )
    ####


def parse_mission_composition_response(payload: object) -> MissionCompositionRunResponse:
    """Validate a serialized or in-memory common runner response."""

    return _RESPONSE_ADAPTER.validate_python(payload)
    ####


def _validate_executor_result(
    request: MissionCompositionRunRequest,
    result: MissionCompositionTrajectoryResult,
) -> None:
    mismatches: list[str] = []
    if result.provider_id != request.provider_id:
        mismatches.append("provider_id")
    if result.provider_version != request.provider_version:
        mismatches.append("provider_version")
    if result.request_id != request.request_id:
        mismatches.append("request_id")
    if result.configuration_fingerprint != request.prepared_configuration.fingerprint:
        mismatches.append("configuration_fingerprint")
    if result.primary_model_id != request.model_id:
        mismatches.append("primary_model_id")
    if mismatches:
        raise MissionCompositionExecutionError(
            MissionCompositionDiagnostic(
                severity="error",
                code="provider-result-identity-mismatch",
                message=f"Provider result identity differs from the run request: {mismatches!r}.",
                phase="projection",
                recoverability="fatal",
                provider_id=request.provider_id,
                model_id=request.model_id,
                details={"mismatched_fields": mismatches},
            ),
            category="provider_error",
        )
    ####


def _failure_response(
    request: MissionCompositionRunRequest,
    diagnostic: MissionCompositionDiagnostic,
    *,
    category: FailureCategory,
) -> MissionCompositionFailureResponse:
    retryable = diagnostic.recoverability == "retryable"
    return MissionCompositionFailureResponse(
        failure=MissionCompositionFailure(
            category=category,
            phase=diagnostic.phase,
            operation=request.operation,
            retryable=retryable,
            request_id=request.request_id,
            provider_id=request.provider_id,
            provider_version=request.provider_version,
            model_id=request.model_id,
            configuration_fingerprint=request.prepared_configuration.fingerprint,
            diagnostics=(diagnostic,),
        )
    )
    ####


def _validate_acyclic_lineage(objects: Mapping[str, TrajectoryObject]) -> None:
    for item in objects.values():
        seen: set[str] = set()
        current = item
        while current.parent_object_id is not None:
            if current.object_id in seen:
                raise ValueError("trajectory object lineage must be acyclic")
            seen.add(current.object_id)
            current = objects[current.parent_object_id]
    ####


def _validate_json_output_value(value: Any, *, path: str) -> None:
    """Reject non-JSON and non-finite values before result serialization."""

    if value is None or isinstance(value, bool | str):
        return
    if isinstance(value, int | float):
        if not math.isfinite(float(value)):
            raise ValueError(f"{path} must not contain non-finite numbers")
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{path} object keys must be strings")
            _validate_json_output_value(item, path=f"{path}.{key}")
        return
    if isinstance(value, list | tuple):
        for index, item in enumerate(value):
            _validate_json_output_value(item, path=f"{path}[{index}]")
        return
    raise ValueError(f"{path} has unsupported output type {type(value).__name__!r}")
    ####


def _validate_typed_output_value(
    metadata: TrajectoryChannelMetadata,
    value: Any,
    *,
    path: str,
) -> None:
    """Validate one returned value against advertised type and shape metadata."""

    def visit(item: Any, dimensions: tuple[int | Literal["variable"], ...], item_path: str) -> None:
        if dimensions:
            if not isinstance(item, list | tuple):
                raise ValueError(f"{item_path} must be an array with shape {metadata.shape!r}")
            expected = dimensions[0]
            if expected != "variable" and len(item) != expected:
                raise ValueError(f"{item_path} must contain {expected} items")
            for index, child in enumerate(item):
                visit(child, dimensions[1:], f"{item_path}[{index}]")
            return
        if metadata.data_type == "float64":
            if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
                raise ValueError(f"{item_path} must be a finite number")
        elif metadata.data_type == "int64":
            if isinstance(item, bool) or not isinstance(item, int):
                raise ValueError(f"{item_path} must be an integer")
        elif metadata.data_type == "boolean":
            if not isinstance(item, bool):
                raise ValueError(f"{item_path} must be boolean")
        elif metadata.data_type == "string":
            if not isinstance(item, str):
                raise ValueError(f"{item_path} must be text")
        else:
            _validate_json_output_value(item, path=item_path)
        ####

    visit(value, metadata.shape, path)
    ####


def _audit_diagnostic(
    code: str,
    message: str,
    provider_id: str,
    *,
    model_id: str | None = None,
) -> MissionCompositionDiagnostic:
    return MissionCompositionDiagnostic(
        severity="error",
        code=code,
        message=message,
        phase="discovery",
        recoverability="fatal",
        provider_id=provider_id,
        model_id=model_id,
    )
    ####


def _normalize_code(value: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "-", value.strip().lower()).strip("-")
    return normalized if normalized else "provider-error"
    ####


def _exception_message(error: Exception) -> str:
    text = str(error)
    first = text.split(": ", 1)
    return first[1] if len(first) == 2 else text
    ####


def _json_pointer(path: str) -> str:
    if path.startswith("/"):
        return path
    normalized = re.sub(r"\[([0-9]+)\]", r".\1", path.removeprefix("configuration."))
    parts = (item.replace("~", "~0").replace("/", "~1") for item in normalized.split(".") if item)
    return "/" + "/".join(parts)
    ####


def _validation_error_details(error: ValidationError) -> list[dict[str, object]]:
    """Return stable, JSON-safe Pydantic feedback without echoing full inputs."""

    return [
        {
            "type": str(item["type"]),
            "path": "/" + "/".join(str(part) for part in item["loc"]),
            "message": str(item["msg"]),
        }
        for item in error.errors(include_url=False)
    ]
    ####


__all__ = [
    "DiagnosticPhase",
    "DiagnosticRecoverability",
    "DiagnosticSeverity",
    "FailureCategory",
    "MissionCompositionDiagnostic",
    "MissionCompositionExecutionError",
    "MissionCompositionFailure",
    "MissionCompositionFailureResponse",
    "MissionCompositionOutputSelection",
    "MissionCompositionRunRequest",
    "MissionCompositionRunResponse",
    "MissionCompositionRunnerRegistry",
    "MissionCompositionTrajectoryResponse",
    "MissionCompositionTrajectoryResult",
    "ProviderAdvertisementConformanceReport",
    "ProviderModelAdvertisementConformance",
    "TrajectoryChannelMetadata",
    "TrajectoryEntityRelationship",
    "TrajectoryEvent",
    "TrajectoryObject",
    "TrajectorySample",
    "TrajectorySegmentResult",
    "TrajectoryStateSnapshot",
    "TrajectoryOutcome",
    "TrajectoryObjectStatus",
    "TrajectorySegmentStatus",
    "audit_provider_advertisement",
    "diagnostic_from_exception",
    "parse_mission_composition_response",
    "resolve_output_selection",
]
