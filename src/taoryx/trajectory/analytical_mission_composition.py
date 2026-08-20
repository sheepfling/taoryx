"""Mission Composition provider discovery, composition, and trajectory contracts.

Mission Composition sits above the lower-level :mod:`taoryx.trajectory.providers`
session contract.  A Mission Composition plug-in owns a catalog of vehicles and
advertises enough metadata for a caller to configure a run without knowing
the provider's native model classes.  The host then follows a small,
fail-closed lifecycle:

``metadata -> prepare(request) -> run(prepared) -> standard trajectory``

The models in this module are deliberately provider-neutral.  The
``ExampleMissionCompositionProvider`` at the bottom is an analytical reference
implementation for exercising the contract; it is not a claim about a
historical TAOS model or a source-grounded vehicle.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .configuration_contract import (
    ConfigurationBound,
    ConfigurationChoiceSchema,
    ConfigurationChoiceVariant,
    ConfigurationGroupSchema,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationPeriodicity,
    ConfigurationSequenceSchema,
    ConfigurationValueSpace,
    ControlCommandSemantics,
    NumericPresentationMetadata,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryControlAdvertisement,
    TrajectoryControlAuthorityMetadata,
    TrajectoryControlChannelMetadata,
    TrajectoryControlIntentMetadata,
    TrajectoryControlNativeBindingMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelCapabilities,
    TrajectoryModelMetadata,
    TrajectoryModelPresentationMetadata,
    TrajectoryModelPropertyMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    ValuePresentationMetadata,
    validate_configuration_instance,
)
from .standard_output import StandardEcefState, project_standard_ecef_samples, standard_ecef_state_from_values

MissionCompositionStatus = Literal["declared", "development", "runnable", "deprecated"]
MissionCompositionCapabilityStatus = Literal["native", "emulated", "approximated", "unsupported"]
ParameterValueType = Literal["number", "integer", "boolean", "string", "enum"]
ParameterRole = Literal["initialization", "segment", "constraint"]
TrajectoryStatus = Literal["completed", "terminated", "failed"]
SegmentResultStatus = Literal["completed", "terminated", "skipped"]


class MissionCompositionError(ValueError):
    """Fail-closed diagnostic raised while preparing a Mission Composition request."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        prefix = f"{code}: "
        if field is not None:
            prefix = f"{prefix}{field}: "
        super().__init__(prefix + message)
        ####

    ####


class MissionCompositionParameterValue(BaseModel):
    """One caller-supplied value and its explicitly supplied unit."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    value: Any
    unit: str | None = None


####


class MissionCompositionParameter(BaseModel):
    """Metadata for one initialization or segment parameter.

    Bounds are mathematical validity bounds, not a qualification envelope.
    ``qualified_minimum`` and ``qualified_maximum`` retain the narrower range
    for which a provider has evidence, when such evidence exists.
    """

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    label: str = Field(min_length=1)
    description: str = Field(min_length=1)
    value_type: ParameterValueType = "number"
    canonical_unit: str | None = None
    required: bool = False
    default: Any = None
    default_declared: bool = False
    minimum: float | None = None
    maximum: float | None = None
    qualified_minimum: float | None = None
    qualified_maximum: float | None = None
    choices: tuple[str, ...] = ()
    role: ParameterRole
    frame: str | None = None
    provenance: str = ""

    @model_validator(mode="after")
    def validate_descriptor(self) -> MissionCompositionParameter:
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"parameter {self.id!r} has inverted bounds")
        if self.qualified_minimum is not None and self.qualified_maximum is not None and self.qualified_minimum > self.qualified_maximum:
            raise ValueError(f"parameter {self.id!r} has inverted qualified bounds")
        if self.minimum is not None and self.qualified_minimum is not None and self.qualified_minimum < self.minimum:
            raise ValueError(f"parameter {self.id!r} qualified minimum is below its hard minimum")
        if self.maximum is not None and self.qualified_maximum is not None and self.qualified_maximum > self.maximum:
            raise ValueError(f"parameter {self.id!r} qualified maximum is above its hard maximum")
        if self.value_type == "enum" and not self.choices:
            raise ValueError(f"enum parameter {self.id!r} must declare choices")
        if self.value_type != "enum" and self.choices:
            raise ValueError(f"non-enum parameter {self.id!r} cannot declare choices")
        if self.default_declared:
            self.validate_value(self.default, field=f"parameter {self.id}.default")
        elif self.required:
            # A required value may not silently acquire a default through a
            # caller's interpretation of ``None``.
            pass
        return self
        ####

    def validate_value(self, value: Any, *, field: str) -> Any:
        """Validate one value against this descriptor and return it unchanged."""

        if self.value_type == "number":
            if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
                raise MissionCompositionError("invalid-value", "expected a finite number", field=field)
        elif self.value_type == "integer":
            if isinstance(value, bool) or not isinstance(value, int):
                raise MissionCompositionError("invalid-value", "expected an integer", field=field)
        elif self.value_type == "boolean":
            if not isinstance(value, bool):
                raise MissionCompositionError("invalid-value", "expected a boolean", field=field)
        elif self.value_type in {"string", "enum"}:
            if not isinstance(value, str):
                raise MissionCompositionError("invalid-value", "expected a string", field=field)
            if self.value_type == "enum" and value not in self.choices:
                raise MissionCompositionError("invalid-choice", f"expected one of {list(self.choices)!r}", field=field)
        if isinstance(value, int | float) and not isinstance(value, bool):
            numeric = float(value)
            if self.minimum is not None and numeric < self.minimum:
                raise MissionCompositionError("out-of-bounds", f"must be >= {self.minimum}", field=field)
            if self.maximum is not None and numeric > self.maximum:
                raise MissionCompositionError("out-of-bounds", f"must be <= {self.maximum}", field=field)
        return value
        ####


####


class MissionCompositionInitialization(BaseModel):
    """Advertised initial-state contract for one vehicle model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    parameters: tuple[MissionCompositionParameter, ...] = ()
    provenance: str = ""

    @model_validator(mode="after")
    def validate_parameters(self) -> MissionCompositionInitialization:
        _require_unique_ids(self.parameters, f"initialization {self.id!r}")
        if any(item.role != "initialization" for item in self.parameters):
            raise ValueError(f"initialization {self.id!r} contains a non-initialization parameter")
        return self
        ####


####


class MissionCompositionSegment(BaseModel):
    """Advertised segment type and its sequencing/parameter contract."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    parameters: tuple[MissionCompositionParameter, ...] = ()
    allowed_next: tuple[str, ...] | None = None
    entry_allowed: bool = True
    repeatable: bool = True
    max_occurrences: int | None = Field(default=None, gt=0)
    terminal: bool = True
    provenance: str = ""

    @model_validator(mode="after")
    def validate_parameters(self) -> MissionCompositionSegment:
        _require_unique_ids(self.parameters, f"segment {self.id!r}")
        if any(item.role != "segment" and item.role != "constraint" for item in self.parameters):
            raise ValueError(f"segment {self.id!r} contains an initialization parameter")
        if self.max_occurrences is not None and not self.repeatable:
            raise ValueError(f"segment {self.id!r} cannot set max_occurrences when repeatable is false")
        return self
        ####


####


class MissionCompositionCapability(BaseModel):
    """One capability advertised by a vehicle model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    status: MissionCompositionCapabilityStatus
    description: str = Field(min_length=1)
    fidelities: tuple[str, ...] = Field(min_length=1)
    operations: tuple[Literal["batch", "prepare", "step"], ...] = ("prepare", "batch")
    segment_ids: tuple[str, ...] = ()
    provenance: str = ""


####


class MissionCompositionChannel(BaseModel):
    """Canonical numeric channel emitted in the standard trajectory."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    description: str = Field(min_length=1)
    frame: str | None = None
    provenance: str = ""


####


class MissionCompositionVehicle(BaseModel):
    """Complete discoverable Mission Composition vehicle/model descriptor."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    vehicle_id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    model_kind: str = Field(min_length=1)
    execution_mode: Literal["native", "lowered"] = "native"
    status: MissionCompositionStatus = "declared"
    description: str = Field(min_length=1)
    fidelities: tuple[str, ...] = Field(min_length=1)
    capabilities: tuple[MissionCompositionCapability, ...] = ()
    initializations: tuple[MissionCompositionInitialization, ...] = Field(min_length=1)
    segments: tuple[MissionCompositionSegment, ...] = Field(min_length=1)
    channels: tuple[MissionCompositionChannel, ...] = Field(min_length=1)
    provenance: str = ""

    @model_validator(mode="after")
    def validate_catalog(self) -> MissionCompositionVehicle:
        if len(self.fidelities) != len(set(self.fidelities)):
            raise ValueError(f"vehicle {self.vehicle_id!r} contains duplicate fidelity IDs")
        _require_unique_ids(self.capabilities, f"vehicle {self.vehicle_id!r} capabilities")
        _require_unique_ids(self.initializations, f"vehicle {self.vehicle_id!r} initializations")
        _require_unique_ids(self.segments, f"vehicle {self.vehicle_id!r} segments")
        _require_unique_ids(self.channels, f"vehicle {self.vehicle_id!r} channels")
        segment_ids = {item.id for item in self.segments}
        channel_ids = {item.id for item in self.channels}
        fidelity_ids = set(self.fidelities)
        for initialization in self.initializations:
            unknown_fidelities = sorted(set(initialization.compatible_fidelities) - fidelity_ids)
            if unknown_fidelities:
                raise ValueError(f"initialization {initialization.id!r} references unknown fidelities {unknown_fidelities!r}")
        for segment in self.segments:
            unknown_fidelities = sorted(set(segment.compatible_fidelities) - fidelity_ids)
            if unknown_fidelities:
                raise ValueError(f"segment {segment.id!r} references unknown fidelities {unknown_fidelities!r}")
            if segment.allowed_next is not None:
                unknown = sorted(set(segment.allowed_next) - segment_ids)
                if unknown:
                    raise ValueError(f"segment {segment.id!r} references unknown next segments {unknown!r}")
        for capability in self.capabilities:
            unknown_fidelities = sorted(set(capability.fidelities) - fidelity_ids)
            if unknown_fidelities:
                raise ValueError(f"capability {capability.id!r} references unknown fidelities {unknown_fidelities!r}")
            unknown_segments = sorted(set(capability.segment_ids) - segment_ids)
            if unknown_segments:
                raise ValueError(f"capability {capability.id!r} references unknown segments {unknown_segments!r}")
            if capability.status == "unsupported" and capability.operations:
                raise ValueError(f"unsupported capability {capability.id!r} cannot advertise operations")
        if not channel_ids:
            raise ValueError(f"vehicle {self.vehicle_id!r} must advertise at least one output channel")
        return self
        ####

    def initialization(self, initialization_id: str) -> MissionCompositionInitialization:
        """Return one initialization contract or raise a discovery error."""

        matches = tuple(item for item in self.initializations if item.id == initialization_id)
        if len(matches) != 1:
            raise MissionCompositionError("unknown-initialization", f"vehicle does not advertise {initialization_id!r}", field="initialization_id")
        return matches[0]
        ####

    def segment(self, segment_id: str) -> MissionCompositionSegment:
        """Return one segment contract or raise a discovery error."""

        matches = tuple(item for item in self.segments if item.id == segment_id)
        if len(matches) != 1:
            raise MissionCompositionError("unknown-segment", f"vehicle does not advertise {segment_id!r}", field="segments")
        return matches[0]
        ####


####


class MissionCompositionProviderMetadata(BaseModel):
    """Provider-level publication envelope returned during discovery."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.mission-composition-provider/v1", alias="schema", serialization_alias="schema")
    api_version: str = "1"
    provider_id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    status: MissionCompositionStatus = "runnable"
    description: str = Field(min_length=1)
    supports_batch: bool = True
    supports_prepare: bool = True
    supports_step: bool = False
    output_schema: str = "taoryx.mission-composition-trajectory/v1"
    vehicles: tuple[MissionCompositionVehicle, ...] = Field(min_length=1)
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_vehicles(self) -> MissionCompositionProviderMetadata:
        _require_unique_ids(self.vehicles, f"provider {self.provider_id!r} vehicles")
        if not self.supports_prepare:
            raise ValueError("Mission Composition providers must support preparation")
        return self
        ####

    def vehicle(self, vehicle_id: str) -> MissionCompositionVehicle:
        """Return one advertised vehicle or raise a discovery error."""

        matches = tuple(item for item in self.vehicles if item.vehicle_id == vehicle_id)
        if len(matches) != 1:
            raise MissionCompositionError("unknown-vehicle", f"provider does not advertise {vehicle_id!r}", field="vehicle_id")
        return matches[0]
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the complete provider publication for a UI or client."""

        return self.model_dump(mode="json", by_alias=True)
        ####


####


class MissionCompositionOutputRequest(BaseModel):
    """Output sampling and channel selection requested for a run."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    cadence_s: float = Field(default=0.1, gt=0.0)
    channels: tuple[str, ...] = ()
    include_events: bool = True
    max_samples: int | None = Field(default=None, gt=1)


####


class MissionCompositionSegmentRequest(BaseModel):
    """One occurrence of an advertised segment in the requested sequence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    instance_id: str | None = None
    parameters: dict[str, MissionCompositionParameterValue] = Field(default_factory=dict)


####


class MissionCompositionTrajectoryRequest(BaseModel):
    """Provider-neutral request accepted by the Mission Composition host."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.mission-composition-request/v1", alias="schema", serialization_alias="schema")
    request_id: str = Field(min_length=1)
    vehicle_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    initialization_id: str = Field(min_length=1)
    initialization: dict[str, MissionCompositionParameterValue] = Field(default_factory=dict)
    segments: tuple[MissionCompositionSegmentRequest, ...] = Field(min_length=1)
    output: MissionCompositionOutputRequest = Field(default_factory=MissionCompositionOutputRequest)


####


class MissionCompositionPreparedSegment(BaseModel):
    """Canonical parameter values for one prepared segment occurrence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str
    instance_id: str
    parameters: dict[str, Any] = Field(default_factory=dict)


####


class MissionCompositionPreparedRequest(BaseModel):
    """Immutable, validated handoff from Mission Composition authoring to execution."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.mission-composition-prepared/v1", alias="schema", serialization_alias="schema")
    provider_id: str
    provider_version: str
    request: MissionCompositionTrajectoryRequest
    vehicle_id: str
    fidelity: str
    initialization_id: str
    resolved_initialization: dict[str, Any]
    segments: tuple[MissionCompositionPreparedSegment, ...]
    selected_channels: tuple[str, ...]
    channel_units: dict[str, str | None]
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")

    def canonical_payload(self) -> dict[str, object]:
        """Return the identity-bearing payload excluding its digest."""

        return {
            "schema": self.schema_id,
            "provider_id": self.provider_id,
            "provider_version": self.provider_version,
            "request_id": self.request.request_id,
            "vehicle_id": self.vehicle_id,
            "fidelity": self.fidelity,
            "initialization_id": self.initialization_id,
            "resolved_initialization": self.resolved_initialization,
            "segments": [item.model_dump(mode="json") for item in self.segments],
            "selected_channels": list(self.selected_channels),
            "channel_units": self.channel_units,
            "output": self.request.output.model_dump(mode="json"),
        }
        ####


####


class MissionCompositionTrajectorySample(BaseModel):
    """One accepted truth sample and its required standard ECEF state."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    time_s: float = Field(ge=0.0)
    values: dict[str, float]
    segment_instance_id: str | None = None
    standard_ecef: StandardEcefState

    @model_validator(mode="before")
    @classmethod
    def project_standard_ecef(cls, value: object) -> object:
        """Attach the single-sample ECEF minimum for direct construction."""

        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        if payload.get("standard_ecef") is not None:
            return payload
        time_s = payload.get("time_s")
        values = payload.get("values")
        if isinstance(time_s, int | float) and not isinstance(time_s, bool) and isinstance(values, Mapping):
            payload["standard_ecef"] = standard_ecef_state_from_values(float(time_s), values)
        return payload
        ####


####


class MissionCompositionTrajectoryEvent(BaseModel):
    """Discrete event associated with an accepted trajectory boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    time_s: float = Field(ge=0.0)
    kind: str = Field(min_length=1)
    segment_instance_id: str | None = None
    detail: str = ""


####


class MissionCompositionSegmentResult(BaseModel):
    """Observed span and outcome for one requested segment occurrence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str
    instance_id: str
    start_time_s: float = Field(ge=0.0)
    end_time_s: float = Field(ge=0.0)
    status: SegmentResultStatus
    events: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_span(self) -> MissionCompositionSegmentResult:
        if self.end_time_s < self.start_time_s:
            raise ValueError(f"segment result {self.instance_id!r} ends before it starts")
        return self
        ####


####


class MissionCompositionTrajectory(BaseModel):
    """Standard provider-independent Mission Composition trajectory result."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.mission-composition-trajectory/v1", alias="schema", serialization_alias="schema")
    provider_id: str
    provider_version: str
    request_id: str
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    vehicle_id: str
    fidelity: str
    status: TrajectoryStatus
    samples: tuple[MissionCompositionTrajectorySample, ...] = Field(min_length=1)
    segments: tuple[MissionCompositionSegmentResult, ...] = ()
    events: tuple[MissionCompositionTrajectoryEvent, ...] = ()
    channel_units: dict[str, str | None] = Field(default_factory=dict)
    diagnostics: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="before")
    @classmethod
    def project_standard_ecef(cls, value: object) -> object:
        """Reproject the full accepted history for derivative kinematics."""

        if not isinstance(value, Mapping):
            return value
        payload = dict(value)
        raw_samples = payload.get("samples")
        if not isinstance(raw_samples, Sequence) or isinstance(raw_samples, str | bytes):
            return payload

        source_samples: list[tuple[float, Mapping[str, float]]] = []
        sample_payloads: list[dict[str, Any]] = []
        for sample in raw_samples:
            if isinstance(sample, MissionCompositionTrajectorySample):
                sample_payload = sample.model_dump(mode="python")
            elif isinstance(sample, Mapping):
                sample_payload = dict(sample)
            else:
                return payload
            time_s = sample_payload.get("time_s")
            values = sample_payload.get("values")
            if isinstance(time_s, bool) or not isinstance(time_s, int | float) or not isinstance(values, Mapping):
                return payload
            source_samples.append((float(time_s), values))
            sample_payloads.append(sample_payload)

        if not source_samples:
            return payload
        channel_units = payload.get("channel_units")
        states = project_standard_ecef_samples(
            tuple(source_samples),
            channel_units=channel_units if isinstance(channel_units, Mapping) else None,
        )
        payload["samples"] = tuple(
            {**sample, "standard_ecef": state}
            for sample, state in zip(sample_payloads, states, strict=True)
        )
        return payload
        ####

    @model_validator(mode="after")
    def validate_timeline(self) -> MissionCompositionTrajectory:
        times = tuple(item.time_s for item in self.samples)
        if any(later < earlier for earlier, later in zip(times, times[1:], strict=False)):
            raise ValueError("trajectory samples must be monotonic in accepted time")
        advertised_channels = set(self.channel_units)
        for index, sample in enumerate(self.samples):
            unknown = sorted(set(sample.values) - advertised_channels)
            if unknown:
                raise ValueError(f"sample {index} contains channels absent from channel_units: {unknown!r}")
        return self
        ####

    def to_dict(self) -> dict[str, object]:
        """Return a stable JSON-compatible result envelope."""

        return self.model_dump(mode="json", by_alias=True)
        ####

    def write_json(self, path: str | Path) -> None:
        """Write the standard trajectory artifact deterministically."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(json.dumps(self.to_dict(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
        ####


####


class MissionCompositionProvider(Protocol):
    """Minimal plug-in surface required by the Mission Composition host."""

    @property
    def metadata(self) -> MissionCompositionProviderMetadata:
        """Return the immutable provider publication."""
        ...

    def prepare(self, request: MissionCompositionTrajectoryRequest) -> MissionCompositionPreparedRequest:
        """Validate and canonicalize a request without executing the plant."""
        ...

    def run(self, prepared: MissionCompositionPreparedRequest) -> MissionCompositionTrajectory:
        """Execute one prepared request and return the standard trajectory."""
        ...


####


class MissionCompositionProviderRegistry:
    """Explicit host-side registry for installed Mission Composition plug-ins."""

    def __init__(self, providers: Sequence[MissionCompositionProvider] = ()) -> None:
        self._providers: dict[str, MissionCompositionProvider] = {}
        for provider in providers:
            self.register(provider)
        ####

    ####

    def register(self, provider: MissionCompositionProvider) -> None:
        """Register one provider under its stable publication ID."""

        provider_id = provider.metadata.provider_id
        if provider_id in self._providers:
            raise ValueError(f"duplicate Mission Composition provider {provider_id!r}")
        self._providers[provider_id] = provider
        ####

    def provider(self, provider_id: str) -> MissionCompositionProvider:
        """Select one installed provider or raise an explicit error."""

        try:
            return self._providers[provider_id]
        except KeyError as error:
            raise MissionCompositionError("unknown-provider", f"provider {provider_id!r} is not installed", field="provider_id") from error
        ####

    def metadata(self) -> tuple[MissionCompositionProviderMetadata, ...]:
        """Return provider publications in deterministic ID order."""

        return tuple(self._providers[key].metadata for key in sorted(self._providers))
        ####

    def catalog(self) -> dict[str, object]:
        """Return the host discovery payload for a UI or remote client."""

        return {
            "schema": "taoryx.mission-composition-provider-catalog/v1",
            "providers": [item.public_dict() for item in self.metadata()],
        }
        ####


####


def _require_unique_ids(items: Sequence[BaseModel], scope: str) -> None:
    """Require stable IDs within one advertised collection."""

    ids: list[str] = []
    for item in items:
        identifier = getattr(item, "id", None)
        if identifier is None:
            identifier = getattr(item, "vehicle_id")
        ids.append(str(identifier))
    if len(ids) != len(set(ids)):
        raise ValueError(f"{scope} contains duplicate IDs")
    ####


def _fingerprint(payload: Mapping[str, object]) -> str:
    """Hash a canonical JSON payload."""

    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()
    ####


def _parameter_map(parameters: Sequence[MissionCompositionParameter], scope: str) -> dict[str, MissionCompositionParameter]:
    """Index parameters and preserve duplicate-ID diagnostics."""

    result = {item.id: item for item in parameters}
    if len(result) != len(parameters):
        raise MissionCompositionError("duplicate-parameter", f"{scope} advertises duplicate parameter IDs", field=scope)
    return result
    ####


def _resolve_parameters(
    descriptors: Sequence[MissionCompositionParameter],
    supplied: Mapping[str, MissionCompositionParameterValue],
    *,
    scope: str,
) -> dict[str, Any]:
    """Resolve supplied values and declared defaults in canonical units."""

    descriptor_map = _parameter_map(descriptors, scope)
    unknown = sorted(set(supplied) - set(descriptor_map))
    if unknown:
        raise MissionCompositionError("unknown-parameter", f"not advertised by {scope}: {unknown!r}", field=scope)
    resolved: dict[str, Any] = {}
    for parameter_id, descriptor in descriptor_map.items():
        raw = supplied.get(parameter_id)
        if raw is None:
            if descriptor.default_declared:
                value = descriptor.default
            elif descriptor.required:
                raise MissionCompositionError("missing-parameter", "required value was not supplied", field=f"{scope}.{parameter_id}")
            else:
                continue
        else:
            if raw.unit != descriptor.canonical_unit:
                raise MissionCompositionError(
                    "unit-mismatch",
                    f"expected canonical unit {descriptor.canonical_unit!r}, received {raw.unit!r}",
                    field=f"{scope}.{parameter_id}",
                )
            value = raw.value
        resolved[parameter_id] = descriptor.validate_value(value, field=f"{scope}.{parameter_id}")
    return resolved
    ####


def _example_configuration_schema(vehicle: MissionCompositionVehicle) -> TrajectoryConfigurationSchema:
    """Project one analytical example into the common portable grammar."""

    initialization = ConfigurationChoiceSchema(
        id="initialization",
        label="Initial State",
        description="Select one advertised initial-state contract.",
        variants=tuple(
            ConfigurationChoiceVariant(
                id=item.id,
                label=item.id.replace("_", " ").title(),
                description=item.description,
                compatible_fidelities=item.compatible_fidelities,
                node=ConfigurationGroupSchema(
                    id=f"{item.id}_parameters",
                    label="Initialization Parameters",
                    children=tuple(_example_parameter_schema(parameter) for parameter in item.parameters),
                ),
            )
            for item in vehicle.initializations
        ),
    )
    segment_choice = ConfigurationChoiceSchema(
        id="segment",
        label="Segment Type",
        description="Select one advertised segment for this sequence position.",
        variants=tuple(
            ConfigurationChoiceVariant(
                id=item.id,
                label=item.id.replace("_", " ").title(),
                description=item.description,
                compatible_fidelities=item.compatible_fidelities,
                node=ConfigurationGroupSchema(
                    id=f"{item.id}_parameters",
                    label="Segment Parameters",
                    children=tuple(_example_parameter_schema(parameter) for parameter in item.parameters),
                ),
            )
            for item in vehicle.segments
        ),
    )
    finite_maximums = tuple(item.max_occurrences for item in vehicle.segments if item.max_occurrences is not None)
    return TrajectoryConfigurationSchema(
        model_id=vehicle.vehicle_id,
        model_version=vehicle.version,
        supported_fidelities=vehicle.fidelities,
        root=ConfigurationGroupSchema(
            id="mission",
            label="Mission Configuration",
            description="Initial state followed by a variable-length segment sequence.",
            children=(
                initialization,
                ConfigurationSequenceSchema(
                    id="segments",
                    label="Mission Segments",
                    description="One or more ordered analytical reference segments.",
                    item=segment_choice,
                    minimum_items=1,
                    maximum_items=max(finite_maximums) if len(finite_maximums) == len(vehicle.segments) else None,
                    allow_custom=True,
                ),
            ),
        ),
        claim_boundary=(
            "This schema describes analytical contract fixtures. Validation does not assert historical TAOS behavior or source-grounded vehicle fidelity."
        ),
    )
    ####


def _example_parameter_schema(parameter: MissionCompositionParameter) -> ConfigurationParameterSchema:
    periodic = ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0) if parameter.id.endswith("heading_deg") else None
    interval = None
    if parameter.value_type in {"number", "integer"}:
        interval = ConfigurationInterval(
            minimum=ConfigurationBound(value=parameter.minimum) if parameter.minimum is not None else None,
            maximum=(ConfigurationBound(value=parameter.maximum, inclusive=periodic is None) if parameter.maximum is not None else None),
        )
    value_space = ConfigurationValueSpace(
        topology="periodic_circle" if periodic is not None else "finite_set" if parameter.value_type == "enum" else "euclidean",
        representation="scalar",
        error_rule="wrapped shortest signed difference" if periodic is not None else "exact equality" if parameter.value_type == "enum" else "subtraction",
        interpolation_rule="unwrap then interpolate along the shortest arc"
        if periodic is not None
        else "not interpolable"
        if parameter.value_type == "enum"
        else "linear",
        normalization_rule="wrap to declared principal interval" if periodic is not None else None,
        period=periodic.period if periodic is not None else None,
        equivalence="values separated by integer periods are equivalent" if periodic is not None else None,
        coordinate_chart="S1" if periodic is not None else "R^1" if parameter.value_type in {"number", "integer"} else None,
    )
    quantity_by_unit = {
        "m": "length",
        "m/s": "speed",
        "m/s^2": "acceleration",
        "s": "time",
        "deg": "angle",
        "g0": "load_factor",
    }
    quantity = None if parameter.canonical_unit is None else quantity_by_unit.get(parameter.canonical_unit)
    return ConfigurationParameterSchema(
        id=parameter.id,
        label=parameter.label,
        description=parameter.description,
        value_type=parameter.value_type,
        quantity=quantity,
        canonical_unit=parameter.canonical_unit,
        display_unit=parameter.canonical_unit,
        required=parameter.required,
        default=parameter.default,
        default_declared=parameter.default_declared,
        interval=interval,
        qualified_interval=(
            ConfigurationInterval(
                minimum=ConfigurationBound(value=parameter.qualified_minimum) if parameter.qualified_minimum is not None else None,
                maximum=ConfigurationBound(value=parameter.qualified_maximum) if parameter.qualified_maximum is not None else None,
            )
            if parameter.qualified_minimum is not None or parameter.qualified_maximum is not None
            else None
        ),
        periodicity=periodic,
        choices=parameter.choices,
        role=parameter.role,
        availability="runnable",
        frame="local_ned" if parameter.frame == "NED" else parameter.frame,
        value_space=value_space,
        provenance=parameter.provenance,
    )
    ####


def _example_model_metadata(
    vehicle: MissionCompositionVehicle,
    schema: TrajectoryConfigurationSchema,
) -> TrajectoryModelMetadata:
    mission_template = _analytical_mission_template(vehicle)
    fidelity_labels = {
        "point_mass_3dof": "Point-Mass 3-DOF",
        "pseudo_6dof": "Pseudo 6-DOF",
        "rigid_body_6dof_direct_wrench": "Rigid-Body 6-DOF, Direct Wrench",
        "rigid_body_6dof_surface_allocated": "Rigid-Body 6-DOF, Surface Allocated",
    }
    fidelities = tuple(
        TrajectoryFidelityMetadata(
            id=item,
            label=fidelity_labels.get(item, item),
            rank=index,
            declared=True,
            dynamics_fidelity=("point_mass_3dof" if item == "point_mass_3dof" else "pseudo_6dof" if item == "pseudo_6dof" else "rigid_body_6dof"),
            input_realization=(
                "guidance_command"
                if item in {"point_mass_3dof", "pseudo_6dof"}
                else "direct_wrench"
                if item == "rigid_body_6dof_direct_wrench"
                else "actuator_allocated"
            ),
            actuator_types=("aerodynamic_surfaces",) if item == "rigid_body_6dof_surface_allocated" else ("not_applicable",),
            compatibility_aliases=(item,),
            runtime_fidelity=item,
            control_realization="force_model" if item == "point_mass_3dof" else "unspecified",
            promotion_status="analytical_fixture",
            operations=("validate", "batch", "step"),
            claim_boundary="Analytical reference-fixture fidelity only.",
        )
        for index, item in enumerate(vehicle.fidelities)
    )
    default_channels = (
        "position.north_m",
        "position.east_m",
        "position.altitude_m",
        "velocity.speed_m_s",
        "attitude.heading_deg",
    )
    properties: list[TrajectoryModelPropertyMetadata] = [
        TrajectoryModelPropertyMetadata(
            id="model_kind",
            label="Model Kind",
            description="Provider-authored implementation category.",
            semantic_role="implementation",
            value_type="string",
            value_kind="declared",
            value=vehicle.model_kind,
            value_declared=True,
            presentation=ValuePresentationMetadata(group="identity", order=10),
            provenance=vehicle.provenance,
            claim_boundary="Descriptive provider classification only.",
        ),
        TrajectoryModelPropertyMetadata(
            id="segment_type_count",
            label="Segment Types",
            description="Number of distinct segment contracts published by this analytical model.",
            semantic_role="capability",
            value_type="integer",
            value_kind="exact",
            value=len(vehicle.segments),
            value_declared=True,
            presentation=ValuePresentationMetadata(group="capabilities", order=20),
            provenance="analytical provider catalog",
            claim_boundary="Counts advertised contracts, not mission feasibility.",
        ),
    ]
    if vehicle.vehicle_id == "reference_ballistic_3dof":
        properties.append(
            TrajectoryModelPropertyMetadata(
                id="gravity_acceleration",
                label="Gravity Acceleration",
                description="Constant downward acceleration used by the analytical ballistic propagator.",
                semantic_role="implementation",
                value_type="number",
                value_kind="exact",
                value=9.80665,
                value_declared=True,
                quantity="acceleration",
                canonical_unit="m/s^2",
                display_unit="m/s^2",
                presentation=ValuePresentationMetadata(
                    group="environment",
                    order=30,
                    format=NumericPresentationMetadata(decimal_places=5),
                ),
                provenance="analytical reference implementation constant",
                claim_boundary="Applies only to this deterministic analytical fixture.",
            )
        )
    output_schema = _analytical_output_schema(vehicle, schema)
    return TrajectoryModelMetadata(
        id=vehicle.vehicle_id,
        name=vehicle.display_name,
        version=vehicle.version,
        description=vehicle.description,
        presentation=TrajectoryModelPresentationMetadata(
            display_name=vehicle.display_name,
            short_name=vehicle.display_name.replace("Reference ", ""),
            summary=vehicle.description,
            category="Analytical References",
            subcategory="Native 3-DOF",
            sort_key=vehicle.vehicle_id,
            badges=("Reference", "Runnable"),
            default_fidelity_id=vehicle.fidelities[0],
            default_mission_template_id=mission_template.id,
            default_output_channel_ids=default_channels,
            properties=tuple(properties),
        ),
        model_kind=vehicle.model_kind,
        status=vehicle.status,
        tags=("analytical-reference", vehicle.model_kind),
        execution_capability_profile="taoryx_universal",
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=tuple(item.id for item in vehicle.initializations),
            segment_types=tuple(item.id for item in vehicle.segments),
            termination_modes=("segment_duration", "ground_contact"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=True,
        ),
        realizations=(
            TrajectoryRealizationMetadata(
                id="analytical_point_mass",
                label="Analytical Point-Mass",
                description="Deterministic provider-local analytical realization used to prove the portable contract.",
                status="available",
                dynamics_fidelities=("point_mass_3dof",),
                input_realization="uncontrolled" if vehicle.model_kind == "ballistic_3dof" else "guidance_command",
                controls=_analytical_control_advertisement(vehicle),
                fidelity_aliases=("point_mass_3dof",),
                mission_template_ids=(mission_template.id,),
                operations=("validate", "batch", "step"),
                native_factory_ids=("analytical_reference.v1",),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                claim_boundary="Reference contract fixture only; no physical qualification claim.",
            ),
        ),
        mission_templates=(mission_template,),
        reference_frames=(_analytical_ned_frame(),),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=fidelities,
        provenance=vehicle.provenance,
        claim_boundary="Analytical interface example; no historical or qualification claim.",
    )
    ####


def _analytical_mission_template(
    vehicle: MissionCompositionVehicle,
) -> TrajectoryMissionTemplateMetadata:
    """Advertise the repeatable native sequence behind one analytical runner.

    The reference models accept a caller-authored nonempty repetition of one
    segment kind.  The template records that executable shape and its
    batch and session runner boundary so a common authoring plan can expose the same
    lifecycle as the canonical vehicle providers without turning the fixture
    into a physical mission claim.
    """

    initialization = vehicle.initializations[0]
    segment = vehicle.segments[0]
    identifier = f"{vehicle.vehicle_id}_repeatable_sequence_v1"
    return TrajectoryMissionTemplateMetadata(
        id=identifier,
        name=f"Repeatable {segment.id.replace('_', ' ').title()} Sequence",
        description=(
            f"One or more native {segment.id!r} segments after {initialization.id!r}; "
            "the caller supplies all initial-state and segment values through the published schema."
        ),
        status="runnable_reference",
        initialization_variants=(initialization.id,),
        segment_sequence=(segment.id,),
        compatible_fidelities=("point_mass_3dof",),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity="point_mass_3dof",
                realization_id="analytical_point_mass",
                operation="validate",
                status="available",
                execution_mode="schema_validation",
                availability_scope="provider_interface",
                common_runner_status="not_available",
                claim_boundary="Schema validation for the analytical reference fixture only.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity="point_mass_3dof",
                realization_id="analytical_point_mass",
                operation="batch",
                status="available",
                execution_mode="analytical_deterministic",
                availability_scope="provider_interface",
                common_runner_status="registered",
                executor_id="analytical-reference-executor",
                claim_boundary=(
                    "Deterministic analytical reference execution only; no historical, physical-vehicle, or qualification claim."
                ),
            ),
            TrajectoryMissionOperationMetadata(
                fidelity="point_mass_3dof",
                realization_id="analytical_point_mass",
                operation="step",
                status="available",
                execution_mode="provider_session",
                availability_scope="provider_interface",
                common_runner_status="registered",
                executor_id="analytical-reference-session.v1",
                claim_boundary=(
                    "Stateful execution reuses the same analytical transition as batch execution; "
                    "it is a contract fixture, not a physical-vehicle prediction."
                ),
            ),
        ),
        provenance="analytical-reference-fixture",
        claim_boundary=(
            "This template describes only the deterministic fixture's repeatable input shape and lifecycle. "
            "It is not a physical vehicle mission, a controller, or qualification evidence."
        ),
    )
    ####


def _analytical_control_advertisement(
    vehicle: MissionCompositionVehicle,
) -> TrajectoryControlAdvertisement:
    """Describe the explicit open-loop and selectable waypoint session surfaces."""

    if vehicle.model_kind == "ballistic_3dof":
        return TrajectoryControlAdvertisement(
            status="uncontrolled",
            channels=(),
            authorities=(
                TrajectoryControlAuthorityMetadata(
                    id="open_loop_coast",
                    authority="open_loop",
                    availability="available",
                    channel_ids=(),
                    operations=("step",),
                    description="Explicit zero-action authority for the uncontrolled ballistic coast.",
                    command_owner="open_loop",
                    selection_scope="session",
                    switching_policy="locked",
                    scheme_id="open_loop.coast",
                    lowering_chain=("zero_action_frame", "constant_gravity_ballistic_transition"),
                    source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                    provenance="analytical reference implementation",
                    claim_boundary="No external or provider-generated control is applied to ballistic propagation.",
                ),
            ),
            intents=(),
            default_authority_id="open_loop_coast",
            claim_boundary="This realization is an explicit open-loop analytical propagation fixture.",
        )

    channels = _analytical_waypoint_step_channels()
    return TrajectoryControlAdvertisement(
        status="available",
        channels=channels,
        authorities=(
            TrajectoryControlAuthorityMetadata(
                id="configured_waypoint_guidance",
                authority="mission",
                availability="available",
                channel_ids=(),
                operations=("batch", "step"),
                description="Provider-owned steering through the prepared waypoint sequence.",
                command_owner="provider_controller",
                selection_scope="session",
                switching_policy="explicit_bumpless",
                scheme_id="mission.waypoint",
                lowering_chain=("configured_waypoint_sequence", "constant_velocity_geometric_steering"),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Internal geometric steering only; no external action or physical actuator is exposed.",
            ),
            TrajectoryControlAuthorityMetadata(
                id="kinematic_velocity_command",
                authority="kinematic",
                availability="available",
                channel_ids=(
                    "guidance.speed.command",
                    "guidance.heading.command",
                    "guidance.flight_path_angle.command",
                ),
                operations=("step",),
                description="Caller-owned speed, heading, and flight-path-angle commands.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="explicit_bumpless",
                scheme_id="kinematic.flight_path",
                lowering_chain=("kinematic_command_hold", "constant_velocity_cartesian_increment"),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Instantaneous kinematic command tracking; no turn, actuator, or controller dynamics.",
            ),
            TrajectoryControlAuthorityMetadata(
                id="live_waypoint_guidance",
                authority="mission",
                availability="available",
                channel_ids=(
                    "navigation.waypoint.north.command",
                    "navigation.waypoint.east.command",
                    "navigation.waypoint.altitude.command",
                    "navigation.waypoint.capture_radius.command",
                    "navigation.waypoint.speed.command",
                ),
                operations=("step",),
                description="Caller-owned live waypoint and cruise-speed updates.",
                command_owner="caller",
                selection_scope="session",
                switching_policy="explicit_bumpless",
                scheme_id="mission.waypoint",
                lowering_chain=("live_waypoint_hold", "constant_velocity_geometric_steering"),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Geometric waypoint steering only; no physical guidance-loop or actuator claim.",
            ),
        ),
        intents=(
            TrajectoryControlIntentMetadata(
                id="waypoint_tracking",
                label="Waypoint Tracking",
                description="Track each configured local-frame waypoint until capture.",
                resolution="provider_internal",
                segment_ids=("waypoint_leg",),
                mission_template_ids=(),
                channel_ids=(),
                operations=("batch", "step"),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Deterministic provider-local guidance semantics only.",
            ),
            TrajectoryControlIntentMetadata(
                id="kinematic_velocity_control",
                label="Kinematic Velocity Control",
                description="Set straight-line speed and direction at each session boundary.",
                resolution="external_channel",
                segment_ids=("waypoint_leg",),
                mission_template_ids=(),
                channel_ids=(
                    "guidance.speed.command",
                    "guidance.heading.command",
                    "guidance.flight_path_angle.command",
                ),
                operations=("step",),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Direct kinematic state evolution without controller or actuator dynamics.",
            ),
            TrajectoryControlIntentMetadata(
                id="live_waypoint_retargeting",
                label="Live Waypoint Retargeting",
                description="Update the held local waypoint without opening a second stream route.",
                resolution="external_channel",
                segment_ids=("waypoint_leg",),
                mission_template_ids=(),
                channel_ids=(
                    "navigation.waypoint.north.command",
                    "navigation.waypoint.east.command",
                    "navigation.waypoint.altitude.command",
                    "navigation.waypoint.capture_radius.command",
                    "navigation.waypoint.speed.command",
                ),
                operations=("step",),
                source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
                provenance="analytical reference implementation",
                claim_boundary="Live geometric retargeting only.",
            ),
        ),
        default_authority_id="configured_waypoint_guidance",
        claim_boundary=(
            "The configured, direct-kinematic, and live-waypoint profiles share one stateful session; "
            "all remain analytical reference-fixture controls."
        ),
    )
    ####


def _analytical_waypoint_step_channels() -> tuple[TrajectoryControlChannelMetadata, ...]:
    """Return unit-complete action metadata for the two caller-owned profiles."""

    euclidean = ConfigurationValueSpace(
        topology="euclidean",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="linear",
        coordinate_chart="R",
    )
    positive = ConfigurationValueSpace(
        topology="bounded_interval",
        representation="scalar",
        error_rule="subtraction",
        interpolation_rule="linear",
        coordinate_chart="[0, +infinity)",
    )
    periodic = ConfigurationValueSpace(
        topology="periodic_circle",
        representation="scalar",
        error_rule="wrapped signed angular difference",
        interpolation_rule="shortest_arc",
        normalization_rule="wrap_to_principal_interval",
        period=360.0,
        coordinate_chart="[0, 360)",
    )

    def channel(
        identifier: str,
        label: str,
        description: str,
        *,
        quantity: str,
        unit: str,
        value_space: ConfigurationValueSpace,
        lower: float | None = None,
        upper: float | None = None,
        frame: str | None = None,
        order: int,
    ) -> TrajectoryControlChannelMetadata:
        feedback_channel_id = {
            "guidance.speed.command": "velocity.speed_m_s",
            "guidance.heading.command": "attitude.heading_deg",
            "guidance.flight_path_angle.command": "attitude.flight_path_angle_deg",
            "navigation.waypoint.north.command": "position.north_m",
            "navigation.waypoint.east.command": "position.east_m",
            "navigation.waypoint.altitude.command": "position.altitude_m",
            "navigation.waypoint.speed.command": "velocity.speed_m_s",
        }.get(identifier)
        provider_binding = {
            "analytical_session_field": identifier,
            **(
                {"feedback_channel_id": feedback_channel_id}
                if feedback_channel_id is not None
                else {}
            ),
        }
        interval = (
            None
            if lower is None and upper is None
            else ConfigurationInterval(
                minimum=None if lower is None else ConfigurationBound(value=lower),
                maximum=None if upper is None else ConfigurationBound(value=upper),
            )
        )
        return TrajectoryControlChannelMetadata(
            id=identifier,
            label=label,
            description=description,
            channel_kind="action",
            quantity=quantity,
            canonical_unit=unit,
            display_unit=unit,
            interval=interval,
            frame=frame,
            sampling_semantics="held_action",
            value_space=value_space,
            semantics=ControlCommandSemantics(
                value_domain="periodic" if value_space.topology == "periodic_circle" else "continuous",
                command_mode="absolute",
                temporal_semantics="held",
                release_behavior="hold",
                agent_normalization="periodic_wrap" if value_space.topology == "periodic_circle" else "auto",
                agent_clip=interval is not None,
            ),
            availability="available",
            operations=("step",),
            native_channel_id=f"analytical_session.{identifier}",
            native_binding=TrajectoryControlNativeBindingMetadata(
                id=f"analytical_session.{identifier}",
                quantity=quantity,
                canonical_unit=unit,
                interval=interval,
                value_space=value_space,
                semantics=ControlCommandSemantics(
                    value_domain="periodic" if value_space.topology == "periodic_circle" else "continuous",
                    command_mode="absolute",
                    temporal_semantics="held",
                    release_behavior="hold",
                    agent_normalization="periodic_wrap" if value_space.topology == "periodic_circle" else "auto",
                    agent_clip=interval is not None,
                ),
                provider_binding=provider_binding,
            ),
            provider_binding=provider_binding,
            presentation=ValuePresentationMetadata(group="session controls", order=order),
            source_refs=("src/taoryx/trajectory/analytical_mission_composition.py",),
            provenance="analytical reference implementation",
            claim_boundary="Session-only analytical command; no physical control-system claim.",
        )
        ####

    return (
        channel(
            "guidance.speed.command",
            "Speed Command",
            "Held kinematic scalar speed.",
            quantity="speed",
            unit="m/s",
            value_space=positive,
            lower=0.0,
            upper=50_000.0,
            order=10,
        ),
        channel(
            "guidance.heading.command",
            "Heading Command",
            "Held local course bearing clockwise from north.",
            quantity="angle",
            unit="deg",
            value_space=periodic,
            lower=0.0,
            upper=360.0,
            frame="local_ned",
            order=20,
        ),
        channel(
            "guidance.flight_path_angle.command",
            "Flight-Path-Angle Command",
            "Held straight-line flight-path angle, positive upward.",
            quantity="angle",
            unit="deg",
            value_space=euclidean,
            lower=-89.0,
            upper=89.0,
            frame="local_ned",
            order=30,
        ),
        channel(
            "navigation.waypoint.north.command",
            "Waypoint North",
            "Held local-frame north target coordinate.",
            quantity="length",
            unit="m",
            value_space=euclidean,
            frame="local_ned",
            order=40,
        ),
        channel(
            "navigation.waypoint.east.command",
            "Waypoint East",
            "Held local-frame east target coordinate.",
            quantity="length",
            unit="m",
            value_space=euclidean,
            frame="local_ned",
            order=50,
        ),
        channel(
            "navigation.waypoint.altitude.command",
            "Waypoint Altitude",
            "Held geometric altitude target.",
            quantity="length",
            unit="m",
            value_space=positive,
            lower=0.0,
            frame="local_ned",
            order=60,
        ),
        channel(
            "navigation.waypoint.capture_radius.command",
            "Waypoint Capture Radius",
            "Held three-dimensional capture radius.",
            quantity="length",
            unit="m",
            value_space=positive,
            lower=0.1,
            order=70,
        ),
        channel(
            "navigation.waypoint.speed.command",
            "Waypoint Speed",
            "Held geometric steering speed.",
            quantity="speed",
            unit="m/s",
            value_space=positive,
            lower=0.0,
            upper=50_000.0,
            order=80,
        ),
    )
    ####


def _analytical_ned_frame() -> TrajectoryReferenceFrameMetadata:
    return TrajectoryReferenceFrameMetadata(
        id="local_ned",
        name="Local North-East-Down",
        description="Local tangent frame with altitude and vertical-speed channels explicitly positive upward.",
        frame_kind="local_tangent",
        axes=("north", "east", "down"),
        handedness="right",
        origin="provider-selected local launch datum",
        orientation="north-east-down tangent orientation",
        provenance="analytical reference convention",
    )
    ####


def _analytical_output_schema(
    vehicle: MissionCompositionVehicle,
    schema: TrajectoryConfigurationSchema,
) -> TrajectoryOutputSchema:
    channels = tuple(_analytical_output_channel(item, vehicle.fidelities) for item in vehicle.channels)
    telemetry_ids = {"maneuver.load_factor_g"}
    return TrajectoryOutputSchema(
        model_id=schema.model_id,
        model_version=schema.model_version,
        core_channels=tuple(item for item in channels if item.id not in telemetry_ids),
        telemetry_channels=tuple(item for item in channels if item.id in telemetry_ids),
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="maneuver",
                label="Maneuver",
                description="Derived maneuver and load telemetry beyond the canonical kinematic state.",
                channel_ids=("maneuver.load_factor_g",),
                default_selected=True,
                presentation=ValuePresentationMetadata(group="telemetry", order=10),
            ),
        ),
        claim_boundary="Deterministic analytical interface fixture; output metadata is not physical qualification evidence.",
    )
    ####


def _analytical_output_channel(
    channel: MissionCompositionChannel,
    fidelities: tuple[str, ...],
) -> TrajectoryOutputChannelMetadata:
    quantity_by_unit = {
        "m": "length",
        "m/s": "speed",
        "deg": "angle",
        "g0": "load_factor",
    }
    periodic = channel.id == "attitude.heading_deg"
    return TrajectoryOutputChannelMetadata(
        id=channel.id,
        label=channel.id.replace(".", " ").replace("_", " ").title(),
        description=channel.description,
        quantity=quantity_by_unit.get(channel.canonical_unit or ""),
        canonical_unit=channel.canonical_unit,
        display_unit=channel.canonical_unit,
        frame="local_ned" if channel.frame is not None else None,
        interpolation="periodic" if periodic else "linear",
        periodicity=ConfigurationPeriodicity(period=360.0, canonical_minimum=0.0) if periodic else None,
        value_space=(
            ConfigurationValueSpace(
                topology="circle",
                representation="canonical_scalar_angle",
                error_rule="shortest_arc_difference",
                interpolation_rule="shortest_arc",
                normalization_rule="wrap_to_principal_interval",
                period=360.0,
                equivalence="values separated by integer multiples of 360 degrees are equivalent",
            )
            if periodic
            else ConfigurationValueSpace(
                topology="real_line",
                representation="scalar",
                error_rule="absolute_difference",
                interpolation_rule="linear",
            )
        ),
        availability="guaranteed",
        compatible_fidelities=fidelities,
        operations=("batch",),
        presentation=ValuePresentationMetadata(group=channel.id.split(".", maxsplit=1)[0], order=10),
        provenance=channel.provenance or "analytical provider output catalog",
        claim_boundary="Guaranteed for this analytical provider when selected in the output request.",
    )
    ####


class ExampleMissionCompositionProvider:
    """Small deterministic provider used to demonstrate the Mission Composition API.

    The provider exposes two deliberately simple native 3DOF reference models:

    * ``reference_ballistic_3dof`` accepts repeatable ballistic coast segments.
    * ``reference_constant_velocity_waypoint_3dof`` accepts repeatable waypoint
      legs with instantaneous heading changes and constant speed until capture.

    The dynamics are analytical fixtures for contract tests and examples.  A
    source-grounded provider should retain this metadata shape while replacing
    only the preparation/execution implementation and its provenance.
    """

    _OUTPUT_CHANNELS = (
        MissionCompositionChannel(id="position.north_m", canonical_unit="m", description="North position in the local tangent frame", frame="local_ned"),
        MissionCompositionChannel(id="position.east_m", canonical_unit="m", description="East position in the local tangent frame", frame="local_ned"),
        MissionCompositionChannel(id="position.altitude_m", canonical_unit="m", description="Geometric altitude above the local datum", frame="local_ned"),
        MissionCompositionChannel(id="velocity.north_m_s", canonical_unit="m/s", description="North velocity in the local tangent frame", frame="local_ned"),
        MissionCompositionChannel(id="velocity.east_m_s", canonical_unit="m/s", description="East velocity in the local tangent frame", frame="local_ned"),
        MissionCompositionChannel(id="velocity.vertical_m_s", canonical_unit="m/s", description="Upward vertical velocity", frame="local_ned"),
        MissionCompositionChannel(id="velocity.speed_m_s", canonical_unit="m/s", description="Scalar speed"),
        MissionCompositionChannel(id="attitude.heading_deg", canonical_unit="deg", description="Course heading, wrapped to [0, 360)"),
        MissionCompositionChannel(id="attitude.flight_path_angle_deg", canonical_unit="deg", description="Flight-path angle"),
        MissionCompositionChannel(id="maneuver.load_factor_g", canonical_unit="g0", description="Applied load factor magnitude"),
    )

    def __init__(self) -> None:
        common_state_parameters = (
            MissionCompositionParameter(
                id="north_m",
                label="Initial north position",
                description="Initial local tangent-plane north coordinate.",
                canonical_unit="m",
                required=False,
                default=0.0,
                default_declared=True,
                role="initialization",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="east_m",
                label="Initial east position",
                description="Initial local tangent-plane east coordinate.",
                canonical_unit="m",
                required=False,
                default=0.0,
                default_declared=True,
                role="initialization",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="altitude_m",
                label="Initial altitude",
                description="Initial geometric altitude above the local datum.",
                canonical_unit="m",
                required=True,
                minimum=0.0,
                role="initialization",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="speed_m_s",
                label="Initial speed",
                description="Initial scalar speed.",
                canonical_unit="m/s",
                required=True,
                minimum=0.001,
                role="initialization",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="heading_deg",
                label="Initial heading",
                description="Initial course heading measured clockwise from north.",
                canonical_unit="deg",
                required=True,
                minimum=0.0,
                maximum=360.0,
                role="initialization",
                provenance="analytical-reference-fixture",
            ),
        )
        ballistic_state_parameters = (
            *common_state_parameters,
            MissionCompositionParameter(
                id="flight_path_angle_deg",
                label="Initial flight-path angle",
                description="Initial flight-path angle, positive upward.",
                canonical_unit="deg",
                required=False,
                default=0.0,
                default_declared=True,
                minimum=-89.0,
                maximum=89.0,
                role="initialization",
                provenance="analytical-reference-fixture",
            ),
        )
        duration_parameter = MissionCompositionParameter(
            id="duration_s",
            label="Segment duration",
            description="Time for which the segment is propagated.",
            canonical_unit="s",
            required=True,
            minimum=0.001,
            role="segment",
            provenance="analytical-reference-fixture",
        )
        ballistic_coast_parameters = (duration_parameter,)
        waypoint_parameters = (
            duration_parameter,
            MissionCompositionParameter(
                id="waypoint_north_m",
                label="Waypoint north",
                description="Target local tangent-plane north coordinate.",
                canonical_unit="m",
                required=True,
                role="segment",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="waypoint_east_m",
                label="Waypoint east",
                description="Target local tangent-plane east coordinate.",
                canonical_unit="m",
                required=True,
                role="segment",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="waypoint_altitude_m",
                label="Waypoint altitude",
                description="Target geometric altitude.",
                canonical_unit="m",
                required=True,
                minimum=0.0,
                role="segment",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            MissionCompositionParameter(
                id="arrival_tolerance_m",
                label="Arrival tolerance",
                description="Distance at which the waypoint is declared captured.",
                canonical_unit="m",
                required=False,
                default=25.0,
                default_declared=True,
                minimum=0.1,
                role="constraint",
                provenance="analytical-reference-fixture",
            ),
        )
        ballistic = MissionCompositionVehicle(
            vehicle_id="reference_ballistic_3dof",
            display_name="Reference ballistic 3DOF",
            version="1.0.0",
            model_kind="ballistic_3dof",
            execution_mode="native",
            status="runnable",
            description="Deterministic local-frame ballistic model executed directly at the native point-mass 3DOF tier.",
            fidelities=("point_mass_3dof",),
            capabilities=(
                MissionCompositionCapability(
                    id="ballistic-propagation",
                    kind="dynamics",
                    status="native",
                    description="Constant-gravity ballistic coast propagation.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("ballistic_coast",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            initializations=(
                MissionCompositionInitialization(
                    id="launch_state",
                    description="Local tangent-plane position and velocity at launch.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=ballistic_state_parameters,
                    provenance="analytical-reference-fixture",
                ),
            ),
            segments=(
                MissionCompositionSegment(
                    id="ballistic_coast",
                    kind="ballistic_coast",
                    description="Uncontrolled constant-gravity ballistic coast.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=ballistic_coast_parameters,
                    allowed_next=("ballistic_coast",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            channels=self._OUTPUT_CHANNELS,
            provenance="analytical-reference-fixture; not a historical TAOS vehicle",
        )
        waypoint = MissionCompositionVehicle(
            vehicle_id="reference_constant_velocity_waypoint_3dof",
            display_name="Reference constant-velocity waypoint 3DOF",
            version="1.0.0",
            model_kind="constant_velocity_waypoint_3dof",
            execution_mode="native",
            status="runnable",
            description="Deterministic native 3DOF model that flies toward each waypoint at constant speed and holds after capture.",
            fidelities=("point_mass_3dof",),
            capabilities=(
                MissionCompositionCapability(
                    id="point-mass-propagation",
                    kind="dynamics",
                    status="native",
                    description="Constant-speed local-frame point-mass propagation.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("waypoint_leg",),
                    provenance="analytical-reference-fixture",
                ),
                MissionCompositionCapability(
                    id="waypoint-sequencing",
                    kind="guidance",
                    status="native",
                    description="Direct constant-velocity legs to a sequence of waypoint gates.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("waypoint_leg",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            initializations=(
                MissionCompositionInitialization(
                    id="initial_state",
                    description="Local tangent-plane position and horizontal velocity at route start.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=common_state_parameters,
                    provenance="analytical-reference-fixture",
                ),
            ),
            segments=(
                MissionCompositionSegment(
                    id="waypoint_leg",
                    kind="waypoint",
                    description="Fly directly toward one waypoint at constant speed until capture; heading changes at the leg boundary.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=waypoint_parameters,
                    allowed_next=("waypoint_leg",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            channels=self._OUTPUT_CHANNELS,
            provenance="analytical-reference-fixture; not a historical TAOS vehicle",
        )
        self._metadata = MissionCompositionProviderMetadata(
            provider_id="taoryx.example.mission-composition",
            display_name="TAORYX Mission Composition reference provider",
            version="0.1.0",
            status="runnable",
            description="A deterministic analytical plug-in demonstrating Mission Composition discovery and trajectory execution.",
            supports_batch=True,
            supports_prepare=True,
            supports_step=False,
            vehicles=(ballistic, waypoint),
            provenance="repository example",
            claim_boundary=(
                "Contract and lifecycle example only. The analytical models are not source-grounded vehicle models, "
                "historical TAOS runtime compatibility, or qualification evidence."
            ),
        )
        ####

    @property
    def metadata(self) -> MissionCompositionProviderMetadata:
        """Return the provider publication."""

        return self._metadata
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return the common model metadata used by schema-driven consumers."""

        return tuple(_example_model_metadata(vehicle, self.get_model_schema(vehicle.vehicle_id)) for vehicle in self.metadata.vehicles)
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Project one reference vehicle into the portable configuration AST."""

        return _example_configuration_schema(self.metadata.vehicle(model_id))
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Project one reference vehicle's core state and optional telemetry."""

        vehicle = self.metadata.vehicle(model_id)
        return _analytical_output_schema(vehicle, self.get_model_schema(model_id))
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate a portable configuration without running the example model."""

        return validate_configuration_instance(self.get_model_schema(configuration.model_id), configuration)
        ####

    def prepare(self, request: MissionCompositionTrajectoryRequest) -> MissionCompositionPreparedRequest:
        """Validate vehicle, initialization, units, values, and segment graph."""

        vehicle = self.metadata.vehicle(request.vehicle_id)
        if vehicle.status != "runnable":
            raise MissionCompositionError("vehicle-not-runnable", f"vehicle status is {vehicle.status!r}", field="vehicle_id")
        if request.fidelity not in vehicle.fidelities:
            raise MissionCompositionError("unsupported-fidelity", f"vehicle does not advertise {request.fidelity!r}", field="fidelity")
        initialization = vehicle.initialization(request.initialization_id)
        if request.fidelity not in initialization.compatible_fidelities:
            raise MissionCompositionError(
                "initialization-fidelity-mismatch", "initialization is not compatible with requested fidelity", field="initialization_id"
            )
        resolved_initialization = _resolve_parameters(initialization.parameters, request.initialization, scope=f"initialization:{initialization.id}")

        channel_map = {item.id: item for item in vehicle.channels}
        selected_channels = request.output.channels or tuple(channel_map)
        if len(selected_channels) != len(set(selected_channels)):
            raise MissionCompositionError("duplicate-channel", "output channel selection contains duplicates", field="output.channels")
        unknown_channels = sorted(set(selected_channels) - set(channel_map))
        if unknown_channels:
            raise MissionCompositionError("unknown-channel", f"vehicle does not advertise {unknown_channels!r}", field="output.channels")
        channel_units = {channel_id: channel_map[channel_id].canonical_unit for channel_id in selected_channels}

        segment_map = {item.id: item for item in vehicle.segments}
        resolved_segments: list[MissionCompositionPreparedSegment] = []
        seen_instances: set[str] = set()
        occurrence_count: dict[str, int] = {}
        previous: MissionCompositionSegment | None = None
        for index, selected in enumerate(request.segments, start=1):
            try:
                segment = segment_map[selected.id]
            except KeyError as error:
                raise MissionCompositionError("unknown-segment", f"vehicle does not advertise {selected.id!r}", field=f"segments[{index - 1}].id") from error
            if request.fidelity not in segment.compatible_fidelities:
                raise MissionCompositionError(
                    "segment-fidelity-mismatch", "segment is not compatible with requested fidelity", field=f"segments[{index - 1}].id"
                )
            if previous is None:
                if not segment.entry_allowed:
                    raise MissionCompositionError("invalid-sequence", f"segment {segment.id!r} is not an allowed entry segment", field="segments[0].id")
            elif previous.allowed_next is not None and segment.id not in previous.allowed_next:
                raise MissionCompositionError(
                    "invalid-sequence",
                    f"segment {segment.id!r} cannot follow {previous.id!r}; allowed next segments are {list(previous.allowed_next)!r}",
                    field=f"segments[{index - 1}].id",
                )
            occurrence_count[segment.id] = occurrence_count.get(segment.id, 0) + 1
            if not segment.repeatable and occurrence_count[segment.id] > 1:
                raise MissionCompositionError("segment-not-repeatable", f"segment {segment.id!r} may occur only once", field="segments")
            if segment.max_occurrences is not None and occurrence_count[segment.id] > segment.max_occurrences:
                raise MissionCompositionError("segment-occurrence-limit", f"segment {segment.id!r} exceeds max_occurrences", field="segments")
            instance_id = selected.instance_id or f"{index:02d}-{segment.id}"
            if instance_id in seen_instances:
                raise MissionCompositionError("duplicate-instance", f"segment instance {instance_id!r} is repeated", field="segments")
            seen_instances.add(instance_id)
            values = _resolve_parameters(segment.parameters, selected.parameters, scope=f"segment:{instance_id}")
            resolved_segments.append(MissionCompositionPreparedSegment(id=segment.id, instance_id=instance_id, parameters=values))
            previous = segment

        if previous is not None and not previous.terminal:
            raise MissionCompositionError("invalid-sequence", f"final segment {previous.id!r} is not terminal", field="segments")

        if request.output.max_samples is not None:
            estimated_samples = 1 + sum(
                int(math.ceil(float(item.parameters["duration_s"]) / request.output.cadence_s - 1.0e-12))
                for item in resolved_segments
                if "duration_s" in item.parameters
            )
            if estimated_samples > request.output.max_samples:
                raise MissionCompositionError(
                    "output-sample-limit",
                    f"requested cadence would emit about {estimated_samples} samples; limit is {request.output.max_samples}",
                    field="output.max_samples",
                )

        prepared_identity = {
            "provider_id": self.metadata.provider_id,
            "provider_version": self.metadata.version,
            "request_id": request.request_id,
            "vehicle_id": vehicle.vehicle_id,
            "fidelity": request.fidelity,
            "initialization_id": initialization.id,
            "resolved_initialization": resolved_initialization,
            "segments": [item.model_dump(mode="json") for item in resolved_segments],
            "selected_channels": list(selected_channels),
            "channel_units": channel_units,
            "output": request.output.model_dump(mode="json"),
        }
        return MissionCompositionPreparedRequest(
            provider_id=self.metadata.provider_id,
            provider_version=self.metadata.version,
            request=request,
            vehicle_id=vehicle.vehicle_id,
            fidelity=request.fidelity,
            initialization_id=initialization.id,
            resolved_initialization=resolved_initialization,
            segments=tuple(resolved_segments),
            selected_channels=tuple(selected_channels),
            channel_units=channel_units,
            request_fingerprint=_fingerprint(prepared_identity),
        )
        ####

    def run(self, prepared: MissionCompositionPreparedRequest) -> MissionCompositionTrajectory:
        """Execute a prepared analytical request into the common result envelope."""

        if prepared.provider_id != self.metadata.provider_id or prepared.provider_version != self.metadata.version:
            raise MissionCompositionError("provider-mismatch", "prepared request belongs to another provider version")
        expected_fingerprint = _fingerprint(
            {
                "provider_id": prepared.provider_id,
                "provider_version": prepared.provider_version,
                "request_id": prepared.request.request_id,
                "vehicle_id": prepared.vehicle_id,
                "fidelity": prepared.fidelity,
                "initialization_id": prepared.initialization_id,
                "resolved_initialization": prepared.resolved_initialization,
                "segments": [item.model_dump(mode="json") for item in prepared.segments],
                "selected_channels": list(prepared.selected_channels),
                "channel_units": prepared.channel_units,
                "output": prepared.request.output.model_dump(mode="json"),
            }
        )
        if expected_fingerprint != prepared.request_fingerprint:
            raise MissionCompositionError("prepared-request-mutated", "prepared request fingerprint does not match its contents")
        if prepared.vehicle_id == "reference_ballistic_3dof":
            samples, segment_results, events, diagnostics, status = self._run_ballistic(prepared)
        elif prepared.vehicle_id == "reference_constant_velocity_waypoint_3dof":
            samples, segment_results, events, diagnostics, status = self._run_constant_velocity_waypoint(prepared)
        else:
            raise MissionCompositionError("missing-executor", f"no executor is registered for {prepared.vehicle_id!r}")
        return MissionCompositionTrajectory(
            provider_id=self.metadata.provider_id,
            provider_version=self.metadata.version,
            request_id=prepared.request.request_id,
            request_fingerprint=prepared.request_fingerprint,
            vehicle_id=prepared.vehicle_id,
            fidelity=prepared.fidelity,
            status=status,
            samples=tuple(samples),
            segments=tuple(segment_results),
            events=tuple(events) if prepared.request.output.include_events else (),
            channel_units=prepared.channel_units,
            diagnostics=tuple(diagnostics),
            claim_boundary=self.metadata.claim_boundary,
        )
        ####

    def _run_ballistic(
        self, prepared: MissionCompositionPreparedRequest
    ) -> tuple[
        list[MissionCompositionTrajectorySample], list[MissionCompositionSegmentResult], list[MissionCompositionTrajectoryEvent], list[str], TrajectoryStatus
    ]:
        """Propagate the analytical ballistic fixture."""

        state = self._initial_state(prepared.resolved_initialization)
        samples = [self._sample(state, None, prepared.selected_channels)]
        segment_results: list[MissionCompositionSegmentResult] = []
        events: list[MissionCompositionTrajectoryEvent] = []
        diagnostics: list[str] = []
        status: TrajectoryStatus = "completed"
        for segment in prepared.segments:
            start = state["time_s"]
            duration = float(segment.parameters["duration_s"])
            for step in self._steps(duration, prepared.request.output.cadence_s):
                self._propagate_ballistic(state, step)
                samples.append(self._sample(state, segment.instance_id, prepared.selected_channels))
                if state["altitude_m"] <= 0.0:
                    state["altitude_m"] = 0.0
                    status = "terminated"
                    event = MissionCompositionTrajectoryEvent(
                        time_s=state["time_s"],
                        kind="impact",
                        segment_instance_id=segment.instance_id,
                        detail="analytical reference reached the local altitude floor",
                    )
                    events.append(event)
                    diagnostics.append("trajectory terminated at the local altitude floor")
                    break
            segment_results.append(
                MissionCompositionSegmentResult(
                    id=segment.id,
                    instance_id=segment.instance_id,
                    start_time_s=start,
                    end_time_s=state["time_s"],
                    status="terminated" if status == "terminated" else "completed",
                    events=("impact",) if status == "terminated" else (),
                )
            )
            if status == "terminated":
                break
        if len(segment_results) < len(prepared.segments):
            for skipped in prepared.segments[len(segment_results) :]:
                segment_results.append(
                    MissionCompositionSegmentResult(
                        id=skipped.id,
                        instance_id=skipped.instance_id,
                        start_time_s=state["time_s"],
                        end_time_s=state["time_s"],
                        status="skipped",
                    )
                )
        return samples, segment_results, events, diagnostics, status
        ####

    def _run_constant_velocity_waypoint(
        self, prepared: MissionCompositionPreparedRequest
    ) -> tuple[
        list[MissionCompositionTrajectorySample], list[MissionCompositionSegmentResult], list[MissionCompositionTrajectoryEvent], list[str], TrajectoryStatus
    ]:
        """Propagate repeatable constant-velocity waypoint legs."""

        state = self._initial_state(prepared.resolved_initialization)
        samples = [self._sample(state, None, prepared.selected_channels)]
        segment_results: list[MissionCompositionSegmentResult] = []
        events: list[MissionCompositionTrajectoryEvent] = []
        diagnostics: list[str] = []
        for segment in prepared.segments:
            # A capture ends motion for the remainder of its own leg.  The
            # next leg deliberately resumes the configured cruise speed so a
            # route is a real piecewise waypoint sequence rather than a
            # one-shot first-waypoint fixture.
            state["speed_m_s"] = state["cruise_speed_m_s"]
            start = state["time_s"]
            duration = float(segment.parameters["duration_s"])
            segment_events: list[str] = []
            captured = False
            for step in self._steps(duration, prepared.request.output.cadence_s):
                self._propagate_constant_velocity_waypoint(state, segment.parameters, step)
                north_error = float(segment.parameters["waypoint_north_m"]) - state["north_m"]
                east_error = float(segment.parameters["waypoint_east_m"]) - state["east_m"]
                altitude_error = abs(float(segment.parameters["waypoint_altitude_m"]) - state["altitude_m"])
                distance = math.sqrt(north_error * north_error + east_error * east_error + altitude_error * altitude_error)
                if not captured and distance <= float(segment.parameters["arrival_tolerance_m"]):
                    captured = True
                    segment_events.append("waypoint_captured")
                    events.append(
                        MissionCompositionTrajectoryEvent(
                            time_s=state["time_s"],
                            kind="waypoint_captured",
                            segment_instance_id=segment.instance_id,
                            detail=f"captured {segment.instance_id}",
                        )
                    )
                samples.append(self._sample(state, segment.instance_id, prepared.selected_channels))
            events.append(
                MissionCompositionTrajectoryEvent(
                    time_s=state["time_s"],
                    kind="segment_completed",
                    segment_instance_id=segment.instance_id,
                    detail=segment.id,
                )
            )
            segment_events.append("segment_completed")
            segment_results.append(
                MissionCompositionSegmentResult(
                    id=segment.id,
                    instance_id=segment.instance_id,
                    start_time_s=start,
                    end_time_s=state["time_s"],
                    status="completed",
                    events=tuple(segment_events),
                )
            )
        return samples, segment_results, events, diagnostics, "completed"
        ####

    @staticmethod
    def _initial_state(values: Mapping[str, Any]) -> dict[str, float]:
        """Build the private analytical state from resolved initialization."""

        return analytical_initial_state(values)
        ####

    @staticmethod
    def _steps(duration_s: float, cadence_s: float) -> tuple[float, ...]:
        """Return exact positive substeps ending at the requested duration."""

        count = int(math.ceil(duration_s / cadence_s - 1.0e-12))
        return tuple(min(cadence_s, duration_s - index * cadence_s) for index in range(count) if duration_s - index * cadence_s > 1.0e-12)
        ####

    @staticmethod
    def _sample(state: Mapping[str, float], segment_instance_id: str | None, channels: Sequence[str]) -> MissionCompositionTrajectorySample:
        """Project private state into the selected standard channels."""

        heading = math.radians(state["heading_deg"])
        flight_path = math.radians(state["flight_path_angle_deg"])
        horizontal_speed = state["speed_m_s"] * math.cos(flight_path)
        available = {
            "position.north_m": state["north_m"],
            "position.east_m": state["east_m"],
            "position.altitude_m": state["altitude_m"],
            "velocity.north_m_s": horizontal_speed * math.cos(heading),
            "velocity.east_m_s": horizontal_speed * math.sin(heading),
            "velocity.vertical_m_s": state["speed_m_s"] * math.sin(flight_path),
            "velocity.speed_m_s": state["speed_m_s"],
            "attitude.heading_deg": state["heading_deg"] % 360.0,
            "attitude.flight_path_angle_deg": state["flight_path_angle_deg"],
            "maneuver.load_factor_g": state["load_factor_g"],
        }
        return MissionCompositionTrajectorySample(
            time_s=state["time_s"], values={key: available[key] for key in channels}, segment_instance_id=segment_instance_id
        )
        ####

    @staticmethod
    def _propagate_ballistic(state: dict[str, float], duration_s: float) -> None:
        """Advance the constant-gravity ballistic fixture."""

        propagate_ballistic_state(state, duration_s)
        ####

    @staticmethod
    def _propagate_constant_velocity_waypoint(state: dict[str, float], values: Mapping[str, Any], duration_s: float) -> None:
        """Advance in a straight line toward a waypoint at constant speed."""

        propagate_constant_velocity_waypoint_state(state, values, duration_s)
        ####


def analytical_initial_state(values: Mapping[str, Any]) -> dict[str, float]:
    """Build the shared state used by batch and stateful reference execution."""

    return {
        "time_s": 0.0,
        "north_m": float(values.get("north_m", 0.0)),
        "east_m": float(values.get("east_m", 0.0)),
        "altitude_m": float(values["altitude_m"]),
        "speed_m_s": float(values["speed_m_s"]),
        "cruise_speed_m_s": float(values["speed_m_s"]),
        "heading_deg": float(values["heading_deg"]) % 360.0,
        "flight_path_angle_deg": float(values.get("flight_path_angle_deg", 0.0)),
        "load_factor_g": 1.0,
    }
    ####


def propagate_ballistic_state(state: dict[str, float], duration_s: float) -> None:
    """Advance the shared constant-gravity ballistic state in place."""

    heading = math.radians(state["heading_deg"])
    flight_path = math.radians(state["flight_path_angle_deg"])
    horizontal_speed = state["speed_m_s"] * math.cos(flight_path)
    state["north_m"] += horizontal_speed * math.cos(heading) * duration_s
    state["east_m"] += horizontal_speed * math.sin(heading) * duration_s
    state["altitude_m"] += (
        state["speed_m_s"] * math.sin(flight_path) * duration_s
        - 0.5 * 9.80665 * duration_s * duration_s
    )
    vertical_speed = state["speed_m_s"] * math.sin(flight_path) - 9.80665 * duration_s
    state["speed_m_s"] = max(0.0, math.hypot(horizontal_speed, vertical_speed))
    state["flight_path_angle_deg"] = math.degrees(
        math.atan2(vertical_speed, max(horizontal_speed, 1.0e-12))
    )
    state["load_factor_g"] = 1.0
    state["time_s"] += duration_s
    ####


def propagate_constant_velocity_waypoint_state(
    state: dict[str, float],
    values: Mapping[str, Any],
    duration_s: float,
) -> None:
    """Advance the shared straight-line constant-speed waypoint state."""

    north_error = float(values["waypoint_north_m"]) - state["north_m"]
    east_error = float(values["waypoint_east_m"]) - state["east_m"]
    altitude_error = float(values["waypoint_altitude_m"]) - state["altitude_m"]
    distance = math.sqrt(north_error * north_error + east_error * east_error + altitude_error * altitude_error)
    speed = state["speed_m_s"]
    if distance <= 1.0e-12 or speed <= 1.0e-12:
        state["speed_m_s"] = 0.0
        state["flight_path_angle_deg"] = 0.0
        state["load_factor_g"] = 1.0
        state["time_s"] += duration_s
        return

    horizontal_distance = math.hypot(north_error, east_error)
    state["heading_deg"] = math.degrees(math.atan2(east_error, north_error)) % 360.0
    state["flight_path_angle_deg"] = math.degrees(math.atan2(altitude_error, horizontal_distance))
    travel = min(speed * duration_s, distance)
    fraction = travel / distance
    state["north_m"] += north_error * fraction
    state["east_m"] += east_error * fraction
    state["altitude_m"] += altitude_error * fraction
    if travel >= distance:
        state["speed_m_s"] = 0.0
        state["flight_path_angle_deg"] = 0.0
    state["load_factor_g"] = 1.0
    state["time_s"] += duration_s
    ####


__all__ = [
    "MissionCompositionCapabilityStatus",
    "MissionCompositionCapability",
    "MissionCompositionChannel",
    "MissionCompositionError",
    "MissionCompositionInitialization",
    "MissionCompositionParameter",
    "MissionCompositionParameterValue",
    "MissionCompositionPreparedRequest",
    "MissionCompositionPreparedSegment",
    "MissionCompositionProvider",
    "MissionCompositionProviderMetadata",
    "MissionCompositionProviderRegistry",
    "MissionCompositionSegment",
    "MissionCompositionSegmentRequest",
    "MissionCompositionSegmentResult",
    "MissionCompositionStatus",
    "MissionCompositionTrajectory",
    "MissionCompositionTrajectoryEvent",
    "MissionCompositionTrajectoryRequest",
    "MissionCompositionTrajectorySample",
    "MissionCompositionOutputRequest",
    "ExampleMissionCompositionProvider",
    "analytical_initial_state",
    "propagate_ballistic_state",
    "propagate_constant_velocity_waypoint_state",
]
####
