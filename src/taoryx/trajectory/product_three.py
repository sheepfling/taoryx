"""Product 3 provider discovery, composition, and trajectory contracts.

Product 3 sits above the lower-level :mod:`taoryx.trajectory.providers`
session contract.  A Product 3 plug-in owns a catalog of vehicles and
advertises enough metadata for a caller to configure a run without knowing
the provider's native model classes.  The host then follows a small,
fail-closed lifecycle:

``metadata -> prepare(request) -> run(prepared) -> standard trajectory``

The models in this module are deliberately provider-neutral.  The
``ExampleProductThreeProvider`` at the bottom is an analytical reference
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

ProductThreeStatus = Literal["declared", "development", "runnable", "deprecated"]
ProductThreeCapabilityStatus = Literal["native", "emulated", "approximated", "unsupported"]
ParameterValueType = Literal["number", "integer", "boolean", "string", "enum"]
ParameterRole = Literal["initialization", "segment", "constraint"]
TrajectoryStatus = Literal["completed", "terminated", "failed"]
SegmentResultStatus = Literal["completed", "terminated", "skipped"]


class ProductThreeError(ValueError):
    """Fail-closed diagnostic raised while preparing a Product 3 request."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        prefix = f"{code}: "
        if field is not None:
            prefix = f"{prefix}{field}: "
        super().__init__(prefix + message)
        ####
    ####


class ProductThreeParameterValue(BaseModel):
    """One caller-supplied value and its explicitly supplied unit."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    value: Any
    unit: str | None = None
####


class ProductThreeParameter(BaseModel):
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
    def validate_descriptor(self) -> ProductThreeParameter:
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
                raise ProductThreeError("invalid-value", "expected a finite number", field=field)
        elif self.value_type == "integer":
            if isinstance(value, bool) or not isinstance(value, int):
                raise ProductThreeError("invalid-value", "expected an integer", field=field)
        elif self.value_type == "boolean":
            if not isinstance(value, bool):
                raise ProductThreeError("invalid-value", "expected a boolean", field=field)
        elif self.value_type in {"string", "enum"}:
            if not isinstance(value, str):
                raise ProductThreeError("invalid-value", "expected a string", field=field)
            if self.value_type == "enum" and value not in self.choices:
                raise ProductThreeError("invalid-choice", f"expected one of {list(self.choices)!r}", field=field)
        if isinstance(value, int | float) and not isinstance(value, bool):
            numeric = float(value)
            if self.minimum is not None and numeric < self.minimum:
                raise ProductThreeError("out-of-bounds", f"must be >= {self.minimum}", field=field)
            if self.maximum is not None and numeric > self.maximum:
                raise ProductThreeError("out-of-bounds", f"must be <= {self.maximum}", field=field)
        return value
        ####
####


class ProductThreeInitialization(BaseModel):
    """Advertised initial-state contract for one vehicle model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    parameters: tuple[ProductThreeParameter, ...] = ()
    provenance: str = ""

    @model_validator(mode="after")
    def validate_parameters(self) -> ProductThreeInitialization:
        _require_unique_ids(self.parameters, f"initialization {self.id!r}")
        if any(item.role != "initialization" for item in self.parameters):
            raise ValueError(f"initialization {self.id!r} contains a non-initialization parameter")
        return self
        ####
####


class ProductThreeSegment(BaseModel):
    """Advertised segment type and its sequencing/parameter contract."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    description: str = Field(min_length=1)
    compatible_fidelities: tuple[str, ...] = Field(min_length=1)
    parameters: tuple[ProductThreeParameter, ...] = ()
    allowed_next: tuple[str, ...] | None = None
    entry_allowed: bool = True
    repeatable: bool = True
    max_occurrences: int | None = Field(default=None, gt=0)
    terminal: bool = True
    provenance: str = ""

    @model_validator(mode="after")
    def validate_parameters(self) -> ProductThreeSegment:
        _require_unique_ids(self.parameters, f"segment {self.id!r}")
        if any(item.role != "segment" and item.role != "constraint" for item in self.parameters):
            raise ValueError(f"segment {self.id!r} contains an initialization parameter")
        if self.max_occurrences is not None and not self.repeatable:
            raise ValueError(f"segment {self.id!r} cannot set max_occurrences when repeatable is false")
        return self
        ####
####


class ProductThreeCapability(BaseModel):
    """One capability advertised by a vehicle model."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    kind: str = Field(min_length=1)
    status: ProductThreeCapabilityStatus
    description: str = Field(min_length=1)
    fidelities: tuple[str, ...] = Field(min_length=1)
    operations: tuple[Literal["batch", "prepare", "step"], ...] = ("prepare", "batch")
    segment_ids: tuple[str, ...] = ()
    provenance: str = ""
####


class ProductThreeChannel(BaseModel):
    """Canonical numeric channel emitted in the standard trajectory."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    canonical_unit: str | None = None
    description: str = Field(min_length=1)
    frame: str | None = None
    provenance: str = ""
####


class ProductThreeVehicle(BaseModel):
    """Complete discoverable Product 3 vehicle/model descriptor."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    vehicle_id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    model_kind: str = Field(min_length=1)
    status: ProductThreeStatus = "declared"
    description: str = Field(min_length=1)
    fidelities: tuple[str, ...] = Field(min_length=1)
    capabilities: tuple[ProductThreeCapability, ...] = ()
    initializations: tuple[ProductThreeInitialization, ...] = Field(min_length=1)
    segments: tuple[ProductThreeSegment, ...] = Field(min_length=1)
    channels: tuple[ProductThreeChannel, ...] = Field(min_length=1)
    provenance: str = ""

    @model_validator(mode="after")
    def validate_catalog(self) -> ProductThreeVehicle:
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

    def initialization(self, initialization_id: str) -> ProductThreeInitialization:
        """Return one initialization contract or raise a discovery error."""

        matches = tuple(item for item in self.initializations if item.id == initialization_id)
        if len(matches) != 1:
            raise ProductThreeError("unknown-initialization", f"vehicle does not advertise {initialization_id!r}", field="initialization_id")
        return matches[0]
        ####

    def segment(self, segment_id: str) -> ProductThreeSegment:
        """Return one segment contract or raise a discovery error."""

        matches = tuple(item for item in self.segments if item.id == segment_id)
        if len(matches) != 1:
            raise ProductThreeError("unknown-segment", f"vehicle does not advertise {segment_id!r}", field="segments")
        return matches[0]
        ####
####


class ProductThreeProviderMetadata(BaseModel):
    """Provider-level publication envelope returned during discovery."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.product-three-provider/v1", alias="schema", serialization_alias="schema")
    api_version: str = "1"
    provider_id: str = Field(min_length=1)
    display_name: str = Field(min_length=1)
    version: str = Field(min_length=1)
    status: ProductThreeStatus = "runnable"
    description: str = Field(min_length=1)
    supports_batch: bool = True
    supports_prepare: bool = True
    supports_step: bool = False
    output_schema: str = "taoryx.product-three-trajectory/v1"
    vehicles: tuple[ProductThreeVehicle, ...] = Field(min_length=1)
    provenance: str = ""
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_vehicles(self) -> ProductThreeProviderMetadata:
        _require_unique_ids(self.vehicles, f"provider {self.provider_id!r} vehicles")
        if not self.supports_prepare:
            raise ValueError("Product 3 providers must support preparation")
        return self
        ####

    def vehicle(self, vehicle_id: str) -> ProductThreeVehicle:
        """Return one advertised vehicle or raise a discovery error."""

        matches = tuple(item for item in self.vehicles if item.vehicle_id == vehicle_id)
        if len(matches) != 1:
            raise ProductThreeError("unknown-vehicle", f"provider does not advertise {vehicle_id!r}", field="vehicle_id")
        return matches[0]
        ####

    def public_dict(self) -> dict[str, object]:
        """Return the complete provider publication for a UI or client."""

        return self.model_dump(mode="json", by_alias=True)
        ####
####


class ProductThreeOutputRequest(BaseModel):
    """Output sampling and channel selection requested for a run."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    cadence_s: float = Field(default=0.1, gt=0.0)
    channels: tuple[str, ...] = ()
    include_events: bool = True
    max_samples: int | None = Field(default=None, gt=1)
####


class ProductThreeSegmentRequest(BaseModel):
    """One occurrence of an advertised segment in the requested sequence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str = Field(min_length=1)
    instance_id: str | None = None
    parameters: dict[str, ProductThreeParameterValue] = Field(default_factory=dict)
####


class ProductThreeTrajectoryRequest(BaseModel):
    """Provider-neutral request accepted by the Product 3 host."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.product-three-request/v1", alias="schema", serialization_alias="schema")
    request_id: str = Field(min_length=1)
    vehicle_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    initialization_id: str = Field(min_length=1)
    initialization: dict[str, ProductThreeParameterValue] = Field(default_factory=dict)
    segments: tuple[ProductThreeSegmentRequest, ...] = Field(min_length=1)
    output: ProductThreeOutputRequest = Field(default_factory=ProductThreeOutputRequest)
####


class ProductThreePreparedSegment(BaseModel):
    """Canonical parameter values for one prepared segment occurrence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str
    instance_id: str
    parameters: dict[str, Any] = Field(default_factory=dict)
####


class ProductThreePreparedRequest(BaseModel):
    """Immutable, validated handoff from Product 3 composition to execution."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.product-three-prepared/v1", alias="schema", serialization_alias="schema")
    provider_id: str
    provider_version: str
    request: ProductThreeTrajectoryRequest
    vehicle_id: str
    fidelity: str
    initialization_id: str
    resolved_initialization: dict[str, Any]
    segments: tuple[ProductThreePreparedSegment, ...]
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


class ProductThreeTrajectorySample(BaseModel):
    """One accepted truth sample in the standard trajectory envelope."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    time_s: float = Field(ge=0.0)
    values: dict[str, float]
    segment_instance_id: str | None = None
####


class ProductThreeTrajectoryEvent(BaseModel):
    """Discrete event associated with an accepted trajectory boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    time_s: float = Field(ge=0.0)
    kind: str = Field(min_length=1)
    segment_instance_id: str | None = None
    detail: str = ""
####


class ProductThreeSegmentResult(BaseModel):
    """Observed span and outcome for one requested segment occurrence."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    id: str
    instance_id: str
    start_time_s: float = Field(ge=0.0)
    end_time_s: float = Field(ge=0.0)
    status: SegmentResultStatus
    events: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_span(self) -> ProductThreeSegmentResult:
        if self.end_time_s < self.start_time_s:
            raise ValueError(f"segment result {self.instance_id!r} ends before it starts")
        return self
        ####
####


class ProductThreeTrajectory(BaseModel):
    """Standard provider-independent Product 3 trajectory result."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: str = Field(default="taoryx.product-three-trajectory/v1", alias="schema", serialization_alias="schema")
    provider_id: str
    provider_version: str
    request_id: str
    request_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    vehicle_id: str
    fidelity: str
    status: TrajectoryStatus
    samples: tuple[ProductThreeTrajectorySample, ...] = Field(min_length=1)
    segments: tuple[ProductThreeSegmentResult, ...] = ()
    events: tuple[ProductThreeTrajectoryEvent, ...] = ()
    channel_units: dict[str, str | None] = Field(default_factory=dict)
    diagnostics: tuple[str, ...] = ()
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_timeline(self) -> ProductThreeTrajectory:
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


class ProductThreeProvider(Protocol):
    """Minimal plug-in surface required by the Product 3 host."""

    @property
    def metadata(self) -> ProductThreeProviderMetadata:
        """Return the immutable provider publication."""
        ...

    def prepare(self, request: ProductThreeTrajectoryRequest) -> ProductThreePreparedRequest:
        """Validate and canonicalize a request without executing the plant."""
        ...

    def run(self, prepared: ProductThreePreparedRequest) -> ProductThreeTrajectory:
        """Execute one prepared request and return the standard trajectory."""
        ...
####


class ProductThreeProviderRegistry:
    """Explicit host-side registry for installed Product 3 plug-ins."""

    def __init__(self, providers: Sequence[ProductThreeProvider] = ()) -> None:
        self._providers: dict[str, ProductThreeProvider] = {}
        for provider in providers:
            self.register(provider)
        ####
    ####

    def register(self, provider: ProductThreeProvider) -> None:
        """Register one provider under its stable publication ID."""

        provider_id = provider.metadata.provider_id
        if provider_id in self._providers:
            raise ValueError(f"duplicate Product 3 provider {provider_id!r}")
        self._providers[provider_id] = provider
        ####

    def provider(self, provider_id: str) -> ProductThreeProvider:
        """Select one installed provider or raise an explicit error."""

        try:
            return self._providers[provider_id]
        except KeyError as error:
            raise ProductThreeError("unknown-provider", f"provider {provider_id!r} is not installed", field="provider_id") from error
        ####

    def metadata(self) -> tuple[ProductThreeProviderMetadata, ...]:
        """Return provider publications in deterministic ID order."""

        return tuple(self._providers[key].metadata for key in sorted(self._providers))
        ####

    def catalog(self) -> dict[str, object]:
        """Return the host discovery payload for a UI or remote client."""

        return {
            "schema": "taoryx.product-three-provider-catalog/v1",
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


def _parameter_map(parameters: Sequence[ProductThreeParameter], scope: str) -> dict[str, ProductThreeParameter]:
    """Index parameters and preserve duplicate-ID diagnostics."""

    result = {item.id: item for item in parameters}
    if len(result) != len(parameters):
        raise ProductThreeError("duplicate-parameter", f"{scope} advertises duplicate parameter IDs", field=scope)
    return result
    ####


def _resolve_parameters(
    descriptors: Sequence[ProductThreeParameter],
    supplied: Mapping[str, ProductThreeParameterValue],
    *,
    scope: str,
) -> dict[str, Any]:
    """Resolve supplied values and declared defaults in canonical units."""

    descriptor_map = _parameter_map(descriptors, scope)
    unknown = sorted(set(supplied) - set(descriptor_map))
    if unknown:
        raise ProductThreeError("unknown-parameter", f"not advertised by {scope}: {unknown!r}", field=scope)
    resolved: dict[str, Any] = {}
    for parameter_id, descriptor in descriptor_map.items():
        raw = supplied.get(parameter_id)
        if raw is None:
            if descriptor.default_declared:
                value = descriptor.default
            elif descriptor.required:
                raise ProductThreeError("missing-parameter", "required value was not supplied", field=f"{scope}.{parameter_id}")
            else:
                continue
        else:
            if raw.unit != descriptor.canonical_unit:
                raise ProductThreeError(
                    "unit-mismatch",
                    f"expected canonical unit {descriptor.canonical_unit!r}, received {raw.unit!r}",
                    field=f"{scope}.{parameter_id}",
                )
            value = raw.value
        resolved[parameter_id] = descriptor.validate_value(value, field=f"{scope}.{parameter_id}")
    return resolved
    ####


def _wrap_heading_delta_deg(target: float, current: float) -> float:
    """Return the shortest signed heading difference."""

    return (target - current + 180.0) % 360.0 - 180.0
    ####


def _advance_heading(current: float, target: float, max_rate_rad_s: float, duration_s: float) -> float:
    """Advance a heading under a bounded turn rate."""

    delta = _wrap_heading_delta_deg(target, current)
    maximum_delta = math.degrees(max_rate_rad_s * duration_s)
    if abs(delta) <= maximum_delta:
        return target % 360.0
    return (current + math.copysign(maximum_delta, delta)) % 360.0
    ####


class ExampleProductThreeProvider:
    """Small deterministic provider used to demonstrate the Product 3 API.

    The provider exposes two deliberately simple reference models:

    * ``reference_ballistic`` accepts repeatable coast segments with a fixed
      one-g load limit.
    * ``reference_guided_point_mass`` accepts repeatable waypoint legs and
      bank maneuvers, each with an advertised configurable maximum load factor.

    The dynamics are analytical fixtures for contract tests and examples.  A
    source-grounded provider should retain this metadata shape while replacing
    only the preparation/execution implementation and its provenance.
    """

    _OUTPUT_CHANNELS = (
        ProductThreeChannel(id="position.north_m", canonical_unit="m", description="North position in the local tangent frame", frame="NED"),
        ProductThreeChannel(id="position.east_m", canonical_unit="m", description="East position in the local tangent frame", frame="NED"),
        ProductThreeChannel(id="position.altitude_m", canonical_unit="m", description="Geometric altitude above the local datum", frame="NED"),
        ProductThreeChannel(id="velocity.speed_m_s", canonical_unit="m/s", description="Scalar speed"),
        ProductThreeChannel(id="attitude.heading_deg", canonical_unit="deg", description="Course heading, wrapped to [0, 360)"),
        ProductThreeChannel(id="attitude.flight_path_angle_deg", canonical_unit="deg", description="Flight-path angle"),
        ProductThreeChannel(id="maneuver.load_factor_g", canonical_unit="g0", description="Applied load factor magnitude"),
    )

    def __init__(self) -> None:
        state_parameters = (
            ProductThreeParameter(
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
            ProductThreeParameter(
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
            ProductThreeParameter(
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
            ProductThreeParameter(
                id="speed_m_s",
                label="Initial speed",
                description="Initial scalar speed.",
                canonical_unit="m/s",
                required=True,
                minimum=0.001,
                role="initialization",
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
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
            ProductThreeParameter(
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
        coast_parameters = (
            ProductThreeParameter(
                id="duration_s",
                label="Segment duration",
                description="Time for which the segment is propagated.",
                canonical_unit="s",
                required=True,
                minimum=0.001,
                role="segment",
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
                id="max_load_factor_g",
                label="Maximum load factor",
                description="Maximum allowed load-factor magnitude for this segment.",
                canonical_unit="g0",
                required=False,
                default=1.0,
                default_declared=True,
                minimum=1.0,
                maximum=1.0,
                role="constraint",
                provenance="analytical-reference-fixture",
            ),
        )
        guided_waypoint_parameters = (
            *coast_parameters[:1],
            ProductThreeParameter(
                id="waypoint_north_m",
                label="Waypoint north",
                description="Target local tangent-plane north coordinate.",
                canonical_unit="m",
                required=True,
                role="segment",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
                id="waypoint_east_m",
                label="Waypoint east",
                description="Target local tangent-plane east coordinate.",
                canonical_unit="m",
                required=True,
                role="segment",
                frame="NED",
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
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
            ProductThreeParameter(
                id="max_load_factor_g",
                label="Maximum load factor",
                description="Maximum allowed load-factor magnitude for this maneuver.",
                canonical_unit="g0",
                required=False,
                default=2.5,
                default_declared=True,
                minimum=1.0,
                maximum=6.0,
                role="constraint",
                qualified_minimum=1.0,
                qualified_maximum=4.0,
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
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
        guided_bank_parameters = (
            *coast_parameters[:1],
            ProductThreeParameter(
                id="target_heading_deg",
                label="Target heading",
                description="Heading to acquire through a bounded bank maneuver.",
                canonical_unit="deg",
                required=True,
                minimum=0.0,
                maximum=360.0,
                role="segment",
                provenance="analytical-reference-fixture",
            ),
            ProductThreeParameter(
                id="max_load_factor_g",
                label="Maximum load factor",
                description="Maximum allowed load-factor magnitude for this maneuver.",
                canonical_unit="g0",
                required=False,
                default=2.5,
                default_declared=True,
                minimum=1.0,
                maximum=6.0,
                role="constraint",
                qualified_minimum=1.0,
                qualified_maximum=4.0,
                provenance="analytical-reference-fixture",
            ),
        )
        ballistic = ProductThreeVehicle(
            vehicle_id="reference_ballistic",
            display_name="Reference ballistic point mass",
            version="1.0.0",
            model_kind="ballistic_point_mass",
            status="runnable",
            description="Deterministic local-frame ballistic reference model.",
            fidelities=("point_mass_3dof",),
            capabilities=(
                ProductThreeCapability(
                    id="ballistic-propagation",
                    kind="dynamics",
                    status="native",
                    description="Constant-gravity point-mass coast propagation.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("ballistic_coast",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            initializations=(
                ProductThreeInitialization(
                    id="launch_state",
                    description="Local tangent-plane position and velocity at launch.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=state_parameters,
                    provenance="analytical-reference-fixture",
                ),
            ),
            segments=(
                ProductThreeSegment(
                    id="ballistic_coast",
                    kind="ballistic_coast",
                    description="Uncontrolled ballistic coast with a one-g maneuver limit.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=coast_parameters,
                    allowed_next=("ballistic_coast",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            channels=self._OUTPUT_CHANNELS,
            provenance="analytical-reference-fixture; not a historical TAOS vehicle",
        )
        guided = ProductThreeVehicle(
            vehicle_id="reference_guided_point_mass",
            display_name="Reference guided point mass",
            version="1.0.0",
            model_kind="guided_point_mass_3dof",
            status="runnable",
            description="Deterministic point-mass guidance fixture with repeatable waypoints and bank maneuvers.",
            fidelities=("point_mass_3dof",),
            capabilities=(
                ProductThreeCapability(
                    id="point-mass-propagation",
                    kind="dynamics",
                    status="native",
                    description="Local-frame point-mass propagation.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("coast", "waypoint_leg", "bank_maneuver"),
                    provenance="analytical-reference-fixture",
                ),
                ProductThreeCapability(
                    id="waypoint-guidance",
                    kind="guidance",
                    status="native",
                    description="Bounded turn-rate guidance to a sequence of waypoint gates.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("waypoint_leg",),
                    provenance="analytical-reference-fixture",
                ),
                ProductThreeCapability(
                    id="bank-maneuver",
                    kind="maneuver",
                    status="native",
                    description="Heading change limited by the segment maximum load factor.",
                    fidelities=("point_mass_3dof",),
                    segment_ids=("bank_maneuver",),
                    provenance="analytical-reference-fixture",
                ),
            ),
            initializations=(
                ProductThreeInitialization(
                    id="airborne_state",
                    description="Local tangent-plane position and velocity at the start of a guided route.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=state_parameters,
                    provenance="analytical-reference-fixture",
                ),
            ),
            segments=(
                ProductThreeSegment(
                    id="coast",
                    kind="coast",
                    description="Straight propagation without a waypoint objective.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=coast_parameters,
                    allowed_next=("coast", "waypoint_leg", "bank_maneuver"),
                    provenance="analytical-reference-fixture",
                ),
                ProductThreeSegment(
                    id="waypoint_leg",
                    kind="waypoint",
                    description="Fly toward one waypoint using the declared load-factor limit.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=guided_waypoint_parameters,
                    allowed_next=("waypoint_leg", "bank_maneuver", "coast"),
                    provenance="analytical-reference-fixture",
                ),
                ProductThreeSegment(
                    id="bank_maneuver",
                    kind="bank_maneuver",
                    description="Acquire a target heading with a bounded coordinated-turn rate.",
                    compatible_fidelities=("point_mass_3dof",),
                    parameters=guided_bank_parameters,
                    allowed_next=("waypoint_leg", "bank_maneuver", "coast"),
                    entry_allowed=False,
                    provenance="analytical-reference-fixture",
                ),
            ),
            channels=self._OUTPUT_CHANNELS,
            provenance="analytical-reference-fixture; not a historical TAOS vehicle",
        )
        self._metadata = ProductThreeProviderMetadata(
            provider_id="taoryx.example.product3",
            display_name="TAORYX Product 3 reference provider",
            version="0.1.0",
            status="runnable",
            description="A deterministic analytical plug-in demonstrating Product 3 discovery and trajectory execution.",
            supports_batch=True,
            supports_prepare=True,
            supports_step=False,
            vehicles=(ballistic, guided),
            provenance="repository example",
            claim_boundary=(
                "Contract and lifecycle example only. The analytical models are not source-grounded vehicle models, "
                "historical TAOS runtime compatibility, or qualification evidence."
            ),
        )
        ####

    @property
    def metadata(self) -> ProductThreeProviderMetadata:
        """Return the provider publication."""

        return self._metadata
        ####

    def prepare(self, request: ProductThreeTrajectoryRequest) -> ProductThreePreparedRequest:
        """Validate vehicle, initialization, units, values, and segment graph."""

        vehicle = self.metadata.vehicle(request.vehicle_id)
        if vehicle.status != "runnable":
            raise ProductThreeError("vehicle-not-runnable", f"vehicle status is {vehicle.status!r}", field="vehicle_id")
        if request.fidelity not in vehicle.fidelities:
            raise ProductThreeError("unsupported-fidelity", f"vehicle does not advertise {request.fidelity!r}", field="fidelity")
        initialization = vehicle.initialization(request.initialization_id)
        if request.fidelity not in initialization.compatible_fidelities:
            raise ProductThreeError("initialization-fidelity-mismatch", "initialization is not compatible with requested fidelity", field="initialization_id")
        resolved_initialization = _resolve_parameters(initialization.parameters, request.initialization, scope=f"initialization:{initialization.id}")

        channel_map = {item.id: item for item in vehicle.channels}
        selected_channels = request.output.channels or tuple(channel_map)
        if len(selected_channels) != len(set(selected_channels)):
            raise ProductThreeError("duplicate-channel", "output channel selection contains duplicates", field="output.channels")
        unknown_channels = sorted(set(selected_channels) - set(channel_map))
        if unknown_channels:
            raise ProductThreeError("unknown-channel", f"vehicle does not advertise {unknown_channels!r}", field="output.channels")
        channel_units = {channel_id: channel_map[channel_id].canonical_unit for channel_id in selected_channels}

        segment_map = {item.id: item for item in vehicle.segments}
        resolved_segments: list[ProductThreePreparedSegment] = []
        seen_instances: set[str] = set()
        occurrence_count: dict[str, int] = {}
        previous: ProductThreeSegment | None = None
        for index, selected in enumerate(request.segments, start=1):
            try:
                segment = segment_map[selected.id]
            except KeyError as error:
                raise ProductThreeError("unknown-segment", f"vehicle does not advertise {selected.id!r}", field=f"segments[{index - 1}].id") from error
            if request.fidelity not in segment.compatible_fidelities:
                raise ProductThreeError("segment-fidelity-mismatch", "segment is not compatible with requested fidelity", field=f"segments[{index - 1}].id")
            if previous is None:
                if not segment.entry_allowed:
                    raise ProductThreeError("invalid-sequence", f"segment {segment.id!r} is not an allowed entry segment", field="segments[0].id")
            elif previous.allowed_next is not None and segment.id not in previous.allowed_next:
                raise ProductThreeError(
                    "invalid-sequence",
                    f"segment {segment.id!r} cannot follow {previous.id!r}; allowed next segments are {list(previous.allowed_next)!r}",
                    field=f"segments[{index - 1}].id",
                )
            occurrence_count[segment.id] = occurrence_count.get(segment.id, 0) + 1
            if not segment.repeatable and occurrence_count[segment.id] > 1:
                raise ProductThreeError("segment-not-repeatable", f"segment {segment.id!r} may occur only once", field="segments")
            if segment.max_occurrences is not None and occurrence_count[segment.id] > segment.max_occurrences:
                raise ProductThreeError("segment-occurrence-limit", f"segment {segment.id!r} exceeds max_occurrences", field="segments")
            instance_id = selected.instance_id or f"{index:02d}-{segment.id}"
            if instance_id in seen_instances:
                raise ProductThreeError("duplicate-instance", f"segment instance {instance_id!r} is repeated", field="segments")
            seen_instances.add(instance_id)
            values = _resolve_parameters(segment.parameters, selected.parameters, scope=f"segment:{instance_id}")
            resolved_segments.append(ProductThreePreparedSegment(id=segment.id, instance_id=instance_id, parameters=values))
            previous = segment

        if previous is not None and not previous.terminal:
            raise ProductThreeError("invalid-sequence", f"final segment {previous.id!r} is not terminal", field="segments")

        if request.output.max_samples is not None:
            estimated_samples = 1 + sum(
                int(math.ceil(float(item.parameters["duration_s"]) / request.output.cadence_s - 1.0e-12))
                for item in resolved_segments
                if "duration_s" in item.parameters
            )
            if estimated_samples > request.output.max_samples:
                raise ProductThreeError(
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
        return ProductThreePreparedRequest(
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

    def run(self, prepared: ProductThreePreparedRequest) -> ProductThreeTrajectory:
        """Execute a prepared analytical request into the common result envelope."""

        if prepared.provider_id != self.metadata.provider_id or prepared.provider_version != self.metadata.version:
            raise ProductThreeError("provider-mismatch", "prepared request belongs to another provider version")
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
            raise ProductThreeError("prepared-request-mutated", "prepared request fingerprint does not match its contents")
        if prepared.vehicle_id == "reference_ballistic":
            samples, segment_results, events, diagnostics, status = self._run_ballistic(prepared)
        elif prepared.vehicle_id == "reference_guided_point_mass":
            samples, segment_results, events, diagnostics, status = self._run_guided(prepared)
        else:
            raise ProductThreeError("missing-executor", f"no executor is registered for {prepared.vehicle_id!r}")
        return ProductThreeTrajectory(
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
        self, prepared: ProductThreePreparedRequest
    ) -> tuple[list[ProductThreeTrajectorySample], list[ProductThreeSegmentResult], list[ProductThreeTrajectoryEvent], list[str], TrajectoryStatus]:
        """Propagate the analytical ballistic fixture."""

        state = self._initial_state(prepared.resolved_initialization)
        samples = [self._sample(state, None, prepared.selected_channels)]
        segment_results: list[ProductThreeSegmentResult] = []
        events: list[ProductThreeTrajectoryEvent] = []
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
                    event = ProductThreeTrajectoryEvent(
                        time_s=state["time_s"],
                        kind="impact",
                        segment_instance_id=segment.instance_id,
                        detail="analytical reference reached the local altitude floor",
                    )
                    events.append(event)
                    diagnostics.append("trajectory terminated at the local altitude floor")
                    break
            segment_results.append(
                ProductThreeSegmentResult(
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
                    ProductThreeSegmentResult(
                        id=skipped.id,
                        instance_id=skipped.instance_id,
                        start_time_s=state["time_s"],
                        end_time_s=state["time_s"],
                        status="skipped",
                    )
                )
        return samples, segment_results, events, diagnostics, status
        ####

    def _run_guided(
        self, prepared: ProductThreePreparedRequest
    ) -> tuple[list[ProductThreeTrajectorySample], list[ProductThreeSegmentResult], list[ProductThreeTrajectoryEvent], list[str], TrajectoryStatus]:
        """Propagate waypoint and bank segments with explicit load limits."""

        state = self._initial_state(prepared.resolved_initialization)
        samples = [self._sample(state, None, prepared.selected_channels)]
        segment_results: list[ProductThreeSegmentResult] = []
        events: list[ProductThreeTrajectoryEvent] = []
        diagnostics: list[str] = []
        for segment in prepared.segments:
            start = state["time_s"]
            duration = float(segment.parameters["duration_s"])
            segment_events: list[str] = []
            captured = False
            for step in self._steps(duration, prepared.request.output.cadence_s):
                if segment.id == "waypoint_leg":
                    self._propagate_waypoint(state, segment.parameters, step)
                    north_error = float(segment.parameters["waypoint_north_m"]) - state["north_m"]
                    east_error = float(segment.parameters["waypoint_east_m"]) - state["east_m"]
                    altitude_error = abs(float(segment.parameters["waypoint_altitude_m"]) - state["altitude_m"])
                    distance = math.hypot(north_error, east_error)
                    if not captured and distance <= float(segment.parameters["arrival_tolerance_m"]) and altitude_error <= float(segment.parameters["arrival_tolerance_m"]):
                        captured = True
                        segment_events.append("waypoint_captured")
                        events.append(
                            ProductThreeTrajectoryEvent(
                                time_s=state["time_s"],
                                kind="waypoint_captured",
                                segment_instance_id=segment.instance_id,
                                detail=f"captured {segment.instance_id}",
                            )
                        )
                elif segment.id == "bank_maneuver":
                    self._propagate_bank(state, segment.parameters, step)
                else:
                    self._propagate_guided_coast(state, step)
                samples.append(self._sample(state, segment.instance_id, prepared.selected_channels))
            events.append(
                ProductThreeTrajectoryEvent(
                    time_s=state["time_s"],
                    kind="segment_completed",
                    segment_instance_id=segment.instance_id,
                    detail=segment.id,
                )
            )
            segment_events.append("segment_completed")
            segment_results.append(
                ProductThreeSegmentResult(
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

        return {
            "time_s": 0.0,
            "north_m": float(values.get("north_m", 0.0)),
            "east_m": float(values.get("east_m", 0.0)),
            "altitude_m": float(values["altitude_m"]),
            "speed_m_s": float(values["speed_m_s"]),
            "heading_deg": float(values["heading_deg"]) % 360.0,
            "flight_path_angle_deg": float(values.get("flight_path_angle_deg", 0.0)),
            "load_factor_g": 1.0,
        }
        ####

    @staticmethod
    def _steps(duration_s: float, cadence_s: float) -> tuple[float, ...]:
        """Return exact positive substeps ending at the requested duration."""

        count = int(math.ceil(duration_s / cadence_s - 1.0e-12))
        return tuple(min(cadence_s, duration_s - index * cadence_s) for index in range(count) if duration_s - index * cadence_s > 1.0e-12)
        ####

    @staticmethod
    def _sample(state: Mapping[str, float], segment_instance_id: str | None, channels: Sequence[str]) -> ProductThreeTrajectorySample:
        """Project private state into the selected standard channels."""

        available = {
            "position.north_m": state["north_m"],
            "position.east_m": state["east_m"],
            "position.altitude_m": state["altitude_m"],
            "velocity.speed_m_s": state["speed_m_s"],
            "attitude.heading_deg": state["heading_deg"] % 360.0,
            "attitude.flight_path_angle_deg": state["flight_path_angle_deg"],
            "maneuver.load_factor_g": state["load_factor_g"],
        }
        return ProductThreeTrajectorySample(time_s=state["time_s"], values={key: available[key] for key in channels}, segment_instance_id=segment_instance_id)
        ####

    @staticmethod
    def _propagate_ballistic(state: dict[str, float], duration_s: float) -> None:
        """Advance the constant-gravity ballistic fixture."""

        heading = math.radians(state["heading_deg"])
        flight_path = math.radians(state["flight_path_angle_deg"])
        horizontal_speed = state["speed_m_s"] * math.cos(flight_path)
        state["north_m"] += horizontal_speed * math.cos(heading) * duration_s
        state["east_m"] += horizontal_speed * math.sin(heading) * duration_s
        state["altitude_m"] += state["speed_m_s"] * math.sin(flight_path) * duration_s - 0.5 * 9.80665 * duration_s * duration_s
        vertical_speed = state["speed_m_s"] * math.sin(flight_path) - 9.80665 * duration_s
        state["speed_m_s"] = max(0.0, math.hypot(horizontal_speed, vertical_speed))
        state["flight_path_angle_deg"] = math.degrees(math.atan2(vertical_speed, max(horizontal_speed, 1.0e-12)))
        state["load_factor_g"] = 1.0
        state["time_s"] += duration_s
        ####

    @staticmethod
    def _max_turn_rate(max_load_factor_g: float, speed_m_s: float) -> float:
        """Return a coordinated-turn rate consistent with the advertised g limit."""

        return 9.80665 * math.sqrt(max(0.0, max_load_factor_g * max_load_factor_g - 1.0)) / max(speed_m_s, 1.0e-6)
        ####

    @classmethod
    def _propagate_guided_coast(cls, state: dict[str, float], duration_s: float) -> None:
        """Advance a level guided coast."""

        heading = math.radians(state["heading_deg"])
        state["north_m"] += state["speed_m_s"] * math.cos(heading) * duration_s
        state["east_m"] += state["speed_m_s"] * math.sin(heading) * duration_s
        state["load_factor_g"] = 1.0
        state["time_s"] += duration_s
        ####

    @classmethod
    def _propagate_bank(cls, state: dict[str, float], values: Mapping[str, Any], duration_s: float) -> None:
        """Advance one bounded heading maneuver."""

        limit = float(values["max_load_factor_g"])
        state["heading_deg"] = _advance_heading(
            state["heading_deg"],
            float(values["target_heading_deg"]),
            cls._max_turn_rate(limit, state["speed_m_s"]),
            duration_s,
        )
        cls._propagate_guided_coast(state, duration_s)
        state["load_factor_g"] = min(limit, max(1.0, limit if abs(_wrap_heading_delta_deg(float(values["target_heading_deg"]), state["heading_deg"])) > 1.0e-9 else 1.0))
        ####

    @classmethod
    def _propagate_waypoint(cls, state: dict[str, float], values: Mapping[str, Any], duration_s: float) -> None:
        """Advance toward one waypoint with a bounded turn and altitude response."""

        north_error = float(values["waypoint_north_m"]) - state["north_m"]
        east_error = float(values["waypoint_east_m"]) - state["east_m"]
        target_heading = math.degrees(math.atan2(east_error, north_error)) % 360.0
        limit = float(values["max_load_factor_g"])
        state["heading_deg"] = _advance_heading(
            state["heading_deg"],
            target_heading,
            cls._max_turn_rate(limit, state["speed_m_s"]),
            duration_s,
        )
        target_altitude = float(values["waypoint_altitude_m"])
        altitude_error = target_altitude - state["altitude_m"]
        state["flight_path_angle_deg"] = max(-20.0, min(20.0, math.degrees(math.atan2(altitude_error, max(state["speed_m_s"] * duration_s, 1.0)))))
        heading = math.radians(state["heading_deg"])
        state["north_m"] += state["speed_m_s"] * math.cos(heading) * math.cos(math.radians(state["flight_path_angle_deg"])) * duration_s
        state["east_m"] += state["speed_m_s"] * math.sin(heading) * math.cos(math.radians(state["flight_path_angle_deg"])) * duration_s
        state["altitude_m"] += state["speed_m_s"] * math.sin(math.radians(state["flight_path_angle_deg"])) * duration_s
        state["load_factor_g"] = min(limit, max(1.0, 1.0 + abs(_wrap_heading_delta_deg(target_heading, state["heading_deg"])) / 90.0 * (limit - 1.0)))
        state["time_s"] += duration_s
        ####


__all__ = [
    "ProductThreeCapabilityStatus",
    "ProductThreeCapability",
    "ProductThreeChannel",
    "ProductThreeError",
    "ProductThreeInitialization",
    "ProductThreeParameter",
    "ProductThreeParameterValue",
    "ProductThreePreparedRequest",
    "ProductThreePreparedSegment",
    "ProductThreeProvider",
    "ProductThreeProviderMetadata",
    "ProductThreeProviderRegistry",
    "ProductThreeSegment",
    "ProductThreeSegmentRequest",
    "ProductThreeSegmentResult",
    "ProductThreeStatus",
    "ProductThreeTrajectory",
    "ProductThreeTrajectoryEvent",
    "ProductThreeTrajectoryRequest",
    "ProductThreeTrajectorySample",
    "ProductThreeOutputRequest",
    "ExampleProductThreeProvider",
]
####
