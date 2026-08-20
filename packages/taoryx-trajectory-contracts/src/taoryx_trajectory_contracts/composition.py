"""Provider-neutral discovery, configuration, and batch trajectory contracts."""

from __future__ import annotations

import hashlib
import json
import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .standard import StandardEcefState

CompositionOperation = Literal["validate", "batch", "step"]
OperationAvailability = Literal["available", "conditional", "blocked", "not_available"]
OperationExecutionKind = Literal["native", "adapter", "replay", "provider_defined"]
TrajectoryOutcome = Literal["in_progress", "completed", "terminated", "partial", "failed"]
TrajectoryObjectStatus = Literal["active", "completed", "terminated", "failed"]
DiagnosticSeverity = Literal["error", "warning", "info"]
DiagnosticPhase = Literal["discovery", "configuration", "preflight", "execution", "projection"]


def canonical_json_sha256(value: object) -> str:
    """Return the stable digest used for transport-safe contract handoffs."""

    encoded = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _validate_json_value(value: object, *, path: str) -> None:
    """Reject non-portable values before an interface artifact is emitted."""

    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path} is not JSON-compatible: {error}") from error
    ####


class ContractDiagnostic(BaseModel):
    """Machine-readable feedback returned by any public contract operation."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    severity: DiagnosticSeverity
    code: str = Field(pattern=r"^[a-z0-9]+(?:[-.][a-z0-9]+)*$")
    message: str = Field(min_length=1)
    phase: DiagnosticPhase
    path: str | None = None
    retryable: bool = False
    details: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_details(self) -> ContractDiagnostic:
        _validate_json_value(self.details, path="diagnostic details")
        return self
        ####

    ####


class CompositionOperationDescriptor(BaseModel):
    """Exact operation availability for one mission/fidelity/realization tuple."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mission_template_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    realization_id: str | None = None
    operation: CompositionOperation
    availability: OperationAvailability
    execution_kind: OperationExecutionKind = "provider_defined"
    executor_id: str | None = None
    blockers: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_availability(self) -> CompositionOperationDescriptor:
        if self.availability == "blocked" and not self.blockers:
            raise ValueError("a blocked operation must name at least one blocker")
        if self.availability == "available" and self.blockers:
            raise ValueError("an available operation cannot name blockers")
        if self.executor_id is not None and self.availability == "not_available":
            raise ValueError("an unavailable operation cannot name an executor")
        return self
        ####

    ####


class TrajectoryModelDescriptor(BaseModel):
    """Stable discovery record for one provider-owned trajectory model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-model-descriptor/v1"] = Field(
        default="taoryx.trajectory-model-descriptor/v1",
        alias="schema",
        serialization_alias="schema",
    )
    id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    model_kind: str = Field(min_length=1)
    status: str = Field(min_length=1)
    configuration_schema_id: str = Field(min_length=1)
    output_schema_id: str = Field(min_length=1)
    default_configuration_id: str | None = Field(default=None, min_length=1)
    operations: tuple[CompositionOperationDescriptor, ...] = ()
    tags: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_operations(self) -> TrajectoryModelDescriptor:
        keys = tuple(
            (item.mission_template_id, item.fidelity, item.realization_id, item.operation)
            for item in self.operations
        )
        if len(keys) != len(set(keys)):
            raise ValueError(f"model {self.id!r} contains duplicate operation tuples")
        if len(self.tags) != len(set(self.tags)):
            raise ValueError(f"model {self.id!r} contains duplicate tags")
        return self
        ####

    def supports(
        self,
        operation: CompositionOperation,
        *,
        mission_template_id: str,
        fidelity: str,
        realization_id: str | None = None,
    ) -> bool:
        """Return whether this exact operation tuple is available to a host."""

        return any(
            item.operation == operation
            and item.availability == "available"
            and item.mission_template_id == mission_template_id
            and item.fidelity == fidelity
            and item.realization_id in {None, realization_id}
            for item in self.operations
        )
        ####

    ####


class TrajectoryProviderDescriptor(BaseModel):
    """Discovery envelope a host can consume without loading model code."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-provider-descriptor/v1"] = Field(
        default="taoryx.trajectory-provider-descriptor/v1",
        alias="schema",
        serialization_alias="schema",
    )
    contract_version: Literal["1"] = "1"
    id: str = Field(min_length=1)
    version: str = Field(min_length=1)
    name: str = Field(min_length=1)
    description: str = Field(min_length=1)
    models: tuple[TrajectoryModelDescriptor, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_models(self) -> TrajectoryProviderDescriptor:
        model_ids = tuple(item.id for item in self.models)
        if len(model_ids) != len(set(model_ids)):
            raise ValueError(f"provider {self.id!r} contains duplicate model IDs")
        return self
        ####

    def model(self, model_id: str) -> TrajectoryModelDescriptor:
        """Return one exact model or raise a boundary-safe lookup error."""

        for item in self.models:
            if item.id == model_id:
                return item
        raise KeyError(f"provider {self.id!r} does not advertise model {model_id!r}")
        ####

    ####


class CompositionConfiguration(BaseModel):
    """Opaque provider-owned configuration selected through public discovery.

    The contract standardizes identity, revision, and transport semantics while
    allowing a provider to retain its own typed configuration tree or language
    document in ``payload``.  Hosts must round-trip that payload unchanged.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-configuration/v1"] = Field(
        default="taoryx.trajectory-composition-configuration/v1",
        alias="schema",
        serialization_alias="schema",
    )
    configuration_id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    model_version: str = Field(min_length=1)
    configuration_schema_id: str = Field(min_length=1)
    configuration_schema_fingerprint: str | None = Field(default=None, pattern=r"^[0-9a-f]{64}$")
    fidelity: str = Field(min_length=1)
    realization_id: str | None = None
    mission_template_id: str | None = None
    payload: dict[str, Any]

    @model_validator(mode="after")
    def validate_payload(self) -> CompositionConfiguration:
        _validate_json_value(self.payload, path="configuration payload")
        return self
        ####

    ####


class PreparedCompositionConfiguration(BaseModel):
    """Validated configuration handed from composition to execution.

    ``provider_payload`` is opaque to the host and carries a provider's exact
    resolved/preflight representation.  Its SHA-256 fingerprint is computed
    from both the authored configuration and that provider payload.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-prepared/v1"] = Field(
        default="taoryx.trajectory-composition-prepared/v1",
        alias="schema",
        serialization_alias="schema",
    )
    configuration: CompositionConfiguration
    provider_payload: dict[str, Any]
    fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    diagnostics: tuple[ContractDiagnostic, ...] = ()

    @model_validator(mode="after")
    def validate_fingerprint(self) -> PreparedCompositionConfiguration:
        _validate_json_value(self.provider_payload, path="prepared configuration provider_payload")
        expected = canonical_json_sha256(
            {
                "configuration": self.configuration.model_dump(mode="json", by_alias=True),
                "provider_payload": self.provider_payload,
            }
        )
        if self.fingerprint != expected:
            raise ValueError("prepared configuration fingerprint does not match its contents")
        return self
        ####

    ####


class BatchOutputSelection(BaseModel):
    """Requested common trajectory channels and sampling policy."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    mode: Literal["core", "selected", "all"] = "core"
    cadence_s: float | None = Field(default=None, gt=0.0)
    channels: tuple[str, ...] = ()
    telemetry_groups: tuple[str, ...] = ()
    include_events: bool = True
    include_segments: bool = True
    include_spawned_entities: bool = True
    maximum_samples_per_entity: int | None = Field(default=None, gt=0)
    maximum_entities: int | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def validate_selection(self) -> BatchOutputSelection:
        if len(self.channels) != len(set(self.channels)):
            raise ValueError("batch output selection contains duplicate channel IDs")
        if len(self.telemetry_groups) != len(set(self.telemetry_groups)):
            raise ValueError("batch output selection contains duplicate telemetry-group IDs")
        if self.mode != "selected" and (self.channels or self.telemetry_groups):
            raise ValueError("explicit output channels and telemetry groups require selected output mode")
        return self
        ####

    ####


class BatchRunRequest(BaseModel):
    """Batch execution handoff accepted by a capable composition provider."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-batch-request/v1"] = Field(
        default="taoryx.trajectory-composition-batch-request/v1",
        alias="schema",
        serialization_alias="schema",
    )
    request_id: str = Field(min_length=1)
    prepared_configuration: PreparedCompositionConfiguration
    output: BatchOutputSelection = Field(default_factory=BatchOutputSelection)

    @property
    def provider_id(self) -> str:
        return self.prepared_configuration.configuration.provider_id
        ####

    @property
    def model_id(self) -> str:
        return self.prepared_configuration.configuration.model_id
        ####

    ####


class TrajectorySample(BaseModel):
    """One accepted truth sample with mandatory standard ECEF state."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    time_s: float = Field(ge=0.0)
    values: dict[str, Any]
    segment_instance_id: str | None = None
    standard_ecef: StandardEcefState
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_sample(self) -> TrajectorySample:
        if not math.isfinite(self.time_s):
            raise ValueError("trajectory sample time must be finite")
        _validate_json_value(self.values, path="trajectory sample values")
        _validate_json_value(self.extensions, path="trajectory sample extensions")
        return self
        ####

    ####


class TrajectoryChannel(BaseModel):
    """Portable semantic metadata for one model-specific returned channel."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    unit: str | None = None
    frame: str | None = None
    data_type: Literal["float64", "int64", "boolean", "string", "json"] = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    description: str = ""
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_channel(self) -> TrajectoryChannel:
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"trajectory channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.unit is not None:
            raise ValueError(f"non-numeric trajectory channel {self.id!r} cannot name a unit")
        _validate_json_value(self.extensions, path=f"trajectory channel {self.id!r} extensions")
        return self
        ####

    ####


class TrajectoryEntity(BaseModel):
    """One root or emitted trajectory object in a batch result."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    realization_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    status: TrajectoryObjectStatus
    role: str = Field(min_length=1)
    parent_id: str | None = None
    channels: tuple[TrajectoryChannel, ...] = ()
    samples: tuple[TrajectorySample, ...] = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_history(self) -> TrajectoryEntity:
        channel_ids = tuple(item.id for item in self.channels)
        if len(channel_ids) != len(set(channel_ids)):
            raise ValueError(f"trajectory entity {self.id!r} contains duplicate channel IDs")
        times = tuple(item.time_s for item in self.samples)
        if any(later < earlier for earlier, later in zip(times, times[1:], strict=False)):
            raise ValueError(f"trajectory entity {self.id!r} samples are not monotonic")
        _validate_json_value(self.extensions, path=f"trajectory entity {self.id!r} extensions")
        return self
        ####

    ####


class TrajectoryEvent(BaseModel):
    """One ordered accepted-boundary event in a batch trajectory."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    time_s: float = Field(ge=0.0)
    category: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    entity_id: str = Field(min_length=1)
    parent_entity_id: str | None = None
    detail: str = ""
    data: dict[str, Any] = Field(default_factory=dict)
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_event(self) -> TrajectoryEvent:
        if not math.isfinite(self.time_s):
            raise ValueError("trajectory event time must be finite")
        _validate_json_value(self.data, path="trajectory event data")
        _validate_json_value(self.extensions, path="trajectory event extensions")
        return self
        ####

    ####


class BatchRunResult(BaseModel):
    """Provider-neutral multi-entity batch result with standard ECEF samples."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-batch-result/v1"] = Field(
        default="taoryx.trajectory-composition-batch-result/v1",
        alias="schema",
        serialization_alias="schema",
    )
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    request_id: str = Field(min_length=1)
    configuration_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    primary_entity_id: str = Field(min_length=1)
    status: TrajectoryOutcome
    entities: tuple[TrajectoryEntity, ...] = Field(min_length=1)
    events: tuple[TrajectoryEvent, ...] = ()
    diagnostics: tuple[ContractDiagnostic, ...] = ()
    claim_boundary: str = Field(min_length=1)
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_result(self) -> BatchRunResult:
        entity_ids = tuple(item.id for item in self.entities)
        if len(entity_ids) != len(set(entity_ids)):
            raise ValueError("batch result contains duplicate entity IDs")
        if self.primary_entity_id not in set(entity_ids):
            raise ValueError("primary_entity_id does not identify a returned entity")
        if any(item.entity_id not in set(entity_ids) for item in self.events):
            raise ValueError("batch result event references an unknown entity")
        event_ids = tuple(item.id for item in self.events)
        if len(event_ids) != len(set(event_ids)):
            raise ValueError("batch result contains duplicate event IDs")
        event_times = tuple(item.time_s for item in self.events)
        if any(later < earlier for earlier, later in zip(event_times, event_times[1:], strict=False)):
            raise ValueError("batch result events are not ordered by accepted time")
        _validate_json_value(self.extensions, path="batch result extensions")
        return self
        ####

    ####


__all__ = [
    "BatchOutputSelection",
    "BatchRunRequest",
    "BatchRunResult",
    "CompositionConfiguration",
    "CompositionOperation",
    "CompositionOperationDescriptor",
    "ContractDiagnostic",
    "DiagnosticPhase",
    "DiagnosticSeverity",
    "OperationAvailability",
    "OperationExecutionKind",
    "PreparedCompositionConfiguration",
    "TrajectoryChannel",
    "TrajectoryEntity",
    "TrajectoryEvent",
    "TrajectoryModelDescriptor",
    "TrajectoryObjectStatus",
    "TrajectoryOutcome",
    "TrajectoryProviderDescriptor",
    "TrajectorySample",
    "canonical_json_sha256",
]
