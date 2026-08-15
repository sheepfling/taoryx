"""Stable sensor plug-in, committed-context, and measurement contracts.

The simulation runtime owns causal scheduling, immutable committed inputs,
random streams, delivery, routing, and persistence.  Sensor-family plug-ins
own ideal projection, family physics, configuration, and typed payloads.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field, replace
from types import MappingProxyType
from typing import Any, Generic, NotRequired, Protocol, TypeAlias, TypedDict, TypeVar, cast

import numpy as np
from pydantic import BaseModel

Array3 = np.ndarray
MeasurementT = TypeVar("MeasurementT")
SENSOR_PLUGIN_API_VERSION = "1"


def _vector3(value: Array3, name: str) -> Array3:
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3-vector")
    result = result.copy()
    result.setflags(write=False)
    return result
    ####


def _rotation(value: Array3, name: str) -> Array3:
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(result.T @ result, np.eye(3), atol=1.0e-8) or not np.isclose(
        np.linalg.det(result),
        1.0,
        atol=1.0e-8,
    ):
        raise ValueError(f"{name} must be a proper rotation matrix")
    result = result.copy()
    result.setflags(write=False)
    return result
    ####


def _immutable_value(value: object) -> object:
    if isinstance(value, np.ndarray):
        result = np.asarray(value).copy()
        result.setflags(write=False)
        return result
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _immutable_value(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_immutable_value(item) for item in value)
    if isinstance(value, (set, frozenset)):
        return frozenset(_immutable_value(item) for item in value)
    return value
    ####


def _immutable_mapping(value: Mapping[str, object] | None) -> Mapping[str, object]:
    return MappingProxyType({str(key): _immutable_value(item) for key, item in (value or {}).items()})
    ####


@dataclass(frozen=True, slots=True)
class TruthPoint:
    """One immutable, sensor-visible committed vehicle-truth boundary.

    ``velocity_without_gravity_eci_mps`` is an explicit EOM-produced
    quantity. It is not ``velocity_eci_mps - gravity_eci_mps2``. ECI is the
    navigation frame so attitude changes include Earth rotation when the body
    is fixed to Earth. Translation-only providers may leave attitude and body
    rate unset; those providers expose acceleration channels without claiming
    a physical body frame or gyro measurement.
    """

    time_s: float
    position_eci_m: Array3
    velocity_eci_mps: Array3
    velocity_without_gravity_eci_mps: Array3
    orientation_eci_from_body: Array3 | None
    gravity_eci_mps2: Array3
    angular_rate_body_radps: Array3 | None
    acceleration_eci_mps2: Array3 | None = None
    angular_acceleration_body_radps2: Array3 | None = None
    temperature_celsius: float | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.time_s):
            raise ValueError("truth time must be finite")
        object.__setattr__(self, "position_eci_m", _vector3(self.position_eci_m, "position_eci_m"))
        object.__setattr__(self, "velocity_eci_mps", _vector3(self.velocity_eci_mps, "velocity_eci_mps"))
        object.__setattr__(
            self,
            "velocity_without_gravity_eci_mps",
            _vector3(self.velocity_without_gravity_eci_mps, "velocity_without_gravity_eci_mps"),
        )
        if self.orientation_eci_from_body is not None:
            object.__setattr__(
                self,
                "orientation_eci_from_body",
                _rotation(self.orientation_eci_from_body, "orientation_eci_from_body"),
            )
        object.__setattr__(self, "gravity_eci_mps2", _vector3(self.gravity_eci_mps2, "gravity_eci_mps2"))
        if self.angular_rate_body_radps is not None:
            object.__setattr__(
                self,
                "angular_rate_body_radps",
                _vector3(self.angular_rate_body_radps, "angular_rate_body_radps"),
            )
        if self.acceleration_eci_mps2 is not None:
            object.__setattr__(
                self,
                "acceleration_eci_mps2",
                _vector3(self.acceleration_eci_mps2, "acceleration_eci_mps2"),
            )
        if self.angular_acceleration_body_radps2 is not None:
            object.__setattr__(
                self,
                "angular_acceleration_body_radps2",
                _vector3(self.angular_acceleration_body_radps2, "angular_acceleration_body_radps2"),
            )
        if self.temperature_celsius is not None and not np.isfinite(self.temperature_celsius):
            raise ValueError("temperature_celsius must be finite when supplied")
        ####

    ####


@dataclass(frozen=True, slots=True)
class TruthSegment:
    """Accepted vehicle truth over one interval between committed boundaries."""

    start: TruthPoint
    end: TruthPoint

    def __post_init__(self) -> None:
        if self.end.time_s <= self.start.time_s:
            raise ValueError("truth segment end must be later than its start")
        ####

    ####


@dataclass(frozen=True, slots=True)
class EntityTruth:
    """Committed truth for one scene entity visible to a declared sensor."""

    entity_id: str
    position_eci_m: Array3
    velocity_eci_mps: Array3 = field(default_factory=lambda: np.zeros(3))
    orientation_eci_from_entity: Array3 | None = None
    properties: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.entity_id.strip():
            raise ValueError("sensor-scene entity_id must not be empty")
        object.__setattr__(self, "position_eci_m", _vector3(self.position_eci_m, "entity position_eci_m"))
        object.__setattr__(self, "velocity_eci_mps", _vector3(self.velocity_eci_mps, "entity velocity_eci_mps"))
        if self.orientation_eci_from_entity is not None:
            object.__setattr__(
                self,
                "orientation_eci_from_entity",
                _rotation(self.orientation_eci_from_entity, "orientation_eci_from_entity"),
            )
        object.__setattr__(self, "properties", _immutable_mapping(self.properties))
        ####

    ####


@dataclass(frozen=True, slots=True)
class EnvironmentSnapshot:
    """Immutable environment properties at one committed sensor boundary."""

    properties: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "properties", _immutable_mapping(self.properties))
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorContext:
    """A committed, causally isolated world view supplied to one sensor."""

    snapshot_id: str
    host: TruthPoint
    entities: Mapping[str, EntityTruth] = field(default_factory=dict)
    environment: EnvironmentSnapshot = field(default_factory=EnvironmentSnapshot)

    def __post_init__(self) -> None:
        if not self.snapshot_id.strip():
            raise ValueError("sensor context snapshot_id must not be empty")
        normalized: dict[str, EntityTruth] = {}
        for entity_id, entity in self.entities.items():
            if entity_id != entity.entity_id:
                raise ValueError("sensor context entity key must match EntityTruth.entity_id")
            normalized[entity_id] = entity
        object.__setattr__(self, "entities", MappingProxyType(normalized))
        ####

    @classmethod
    def from_host(cls, host: TruthPoint, *, snapshot_id: str | None = None) -> SensorContext:
        identity = snapshot_id or f"committed@{host.time_s:.17g}"
        return cls(identity, host)
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorContextSegment:
    """Accepted committed context over an interval sensor's sample window."""

    start: SensorContext
    end: SensorContext

    def __post_init__(self) -> None:
        if self.end.host.time_s <= self.start.host.time_s:
            raise ValueError("sensor context segment end must be later than its start")
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorSampleRequest:
    """One point or interval sample request with a runtime-owned RNG stream."""

    point: SensorContext | None = None
    segment: SensorContextSegment | None = None
    rng: np.random.Generator = field(default_factory=np.random.default_rng, compare=False, repr=False)

    def __post_init__(self) -> None:
        if (self.point is None) == (self.segment is None):
            raise ValueError("sensor sample request requires exactly one point or segment")
        ####

    @property
    def sampled_at_s(self) -> float:
        return self.point.host.time_s if self.point is not None else cast(SensorContextSegment, self.segment).end.host.time_s

    @property
    def interval_start_s(self) -> float | None:
        return None if self.segment is None else self.segment.start.host.time_s

    ####


@dataclass(frozen=True, slots=True)
class MeasurementPacket(Generic[MeasurementT]):
    """Timestamped measurement envelope with a family-specific payload."""

    sampled_at_s: float
    available_at_s: float
    interval_start_s: float | None
    payload: MeasurementT | None
    valid: bool = True
    sensor_id: str | None = None
    port: str = "measurement"
    sequence: int | None = None
    schema_id: str | None = None
    invalid_reason: str | None = None

    def __post_init__(self) -> None:
        if not np.isfinite(self.sampled_at_s) or not np.isfinite(self.available_at_s):
            raise ValueError("measurement timestamps must be finite")
        if self.available_at_s < self.sampled_at_s:
            raise ValueError("measurement cannot be available before it is sampled")
        if self.interval_start_s is not None and self.interval_start_s > self.sampled_at_s:
            raise ValueError("measurement interval start cannot be after sample time")
        if self.valid and self.payload is None:
            raise ValueError("valid measurements require a payload")
        if self.sensor_id is not None and not self.sensor_id.strip():
            raise ValueError("measurement sensor_id must not be empty when supplied")
        if not self.port.strip():
            raise ValueError("measurement port must not be empty")
        if self.sequence is not None and self.sequence < 0:
            raise ValueError("measurement sequence must be nonnegative when supplied")
        if self.schema_id is not None and not self.schema_id.strip():
            raise ValueError("measurement schema_id must not be empty when supplied")
        if self.valid and self.invalid_reason is not None:
            raise ValueError("valid measurements cannot declare invalid_reason")
        ####

    ####


PacketPayloadRecord: TypeAlias = dict[str, object]
PacketPayloadContract: TypeAlias = dict[str, object]
PacketProvenanceRecord: TypeAlias = dict[str, object]
PacketProvenanceInput: TypeAlias = Mapping[str, object]


class MeasurementPacketRecord(TypedDict):
    """Stable serialized envelope around one codec-owned sensor payload."""

    sampled_at_s: float
    available_at_s: float
    interval_start_s: float | None
    valid: bool
    payload: PacketPayloadRecord | None
    payload_contract: PacketPayloadContract
    provenance: PacketProvenanceRecord
    sensor_id: NotRequired[str]
    port: NotRequired[str]
    sequence: NotRequired[int]
    schema_id: NotRequired[str]
    invalid_reason: NotRequired[str]


def parse_measurement_packet_record(value: object) -> MeasurementPacketRecord:
    """Normalize an untyped checkpoint/object record into the packet envelope.

    Payload and provenance contents remain codec- or producer-owned, but this
    function owns the outer wire shape and converts compatible mappings to
    ordinary string-keyed dictionaries before a codec consumes them.
    """

    if not isinstance(value, Mapping):
        raise ValueError("measurement packet record must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise ValueError("measurement packet record requires string keys")
    missing = [name for name in ("sampled_at_s", "available_at_s") if name not in value]
    if missing:
        raise ValueError("measurement packet record is missing: " + ", ".join(missing))
    record: dict[str, object] = dict(value)
    record.setdefault("interval_start_s", None)
    record.setdefault("valid", True)
    record.setdefault("payload", None)
    for field_name in ("payload_contract", "provenance"):
        record.setdefault(field_name, {})
    for field_name in ("payload", "payload_contract", "provenance"):
        raw = record[field_name]
        if raw is None and field_name == "payload":
            continue
        if not isinstance(raw, Mapping):
            raise ValueError(f"measurement packet {field_name} must be a mapping")
        if any(not isinstance(key, str) for key in raw):
            raise ValueError(f"measurement packet {field_name} requires string keys")
        record[field_name] = dict(raw)
    return cast(MeasurementPacketRecord, record)
    ####


@dataclass(frozen=True, slots=True)
class ArrayFrameReference:
    """Immutable artifact reference for a large focal-plane or array payload."""

    artifact_id: str
    media_type: str
    shape: tuple[int, ...]
    dtype: str
    sha256: str

    def __post_init__(self) -> None:
        if not self.artifact_id.strip() or not self.media_type.strip() or not self.dtype.strip():
            raise ValueError("array-frame identity, media type, and dtype must not be empty")
        if not self.shape or any(dimension <= 0 for dimension in self.shape):
            raise ValueError("array-frame shape must contain positive dimensions")
        if len(self.sha256) != 64 or any(character not in "0123456789abcdef" for character in self.sha256.casefold()):
            raise ValueError("array-frame sha256 must be a 64-character hexadecimal digest")
        ####

    ####


class CommittedSceneQuery(Protocol):
    """Optional heavy scene-query service bound to one committed snapshot."""

    def trace_ray(
        self,
        snapshot_id: str,
        origin_eci_m: Array3,
        direction_eci: Array3,
        maximum_range_m: float,
    ) -> Mapping[str, object] | None: ...


class SensorArtifactStore(Protocol):
    """Optional storage service for large sensor arrays and rendered frames."""

    def store_array(self, sensor_id: str, sampled_at_s: float, value: np.ndarray) -> ArrayFrameReference: ...


@dataclass(frozen=True, slots=True)
class SensorServices:
    """Runtime services supplied explicitly to plug-ins at construction."""

    scene_query: CommittedSceneQuery | None = None
    artifact_store: SensorArtifactStore | None = None
    extensions: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "extensions", _immutable_mapping(self.extensions))
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorOutputPort:
    name: str
    schema_id: str

    def __post_init__(self) -> None:
        if not self.name.strip() or not self.schema_id.strip():
            raise ValueError("sensor output port name and schema_id must not be empty")
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorPluginManifest:
    """Static capabilities and compatibility of one sensor implementation."""

    kind: str
    family: str
    language_kinds: frozenset[str]
    clock_kind: str
    outputs: tuple[SensorOutputPort, ...]
    required_truth: frozenset[str]
    supported_truth_modes: frozenset[str]
    sample_modes: frozenset[str] = frozenset({"instantaneous"})
    default_cadence_s: float = 0.1
    api_version: str = SENSOR_PLUGIN_API_VERSION
    description: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "outputs", tuple(self.outputs))
        object.__setattr__(self, "language_kinds", frozenset(self.language_kinds))
        object.__setattr__(self, "required_truth", frozenset(self.required_truth))
        object.__setattr__(self, "supported_truth_modes", frozenset(self.supported_truth_modes))
        object.__setattr__(self, "sample_modes", frozenset(self.sample_modes))
        if not self.kind.strip() or not self.family.strip() or not self.clock_kind.strip():
            raise ValueError("sensor plug-in kind, family, and clock_kind must not be empty")
        if not self.language_kinds or any(not kind.strip() for kind in self.language_kinds):
            raise ValueError("sensor plug-in language_kinds must contain nonempty provider-neutral families")
        if self.api_version != SENSOR_PLUGIN_API_VERSION:
            raise ValueError(f"sensor plug-in {self.kind!r} requires API {self.api_version!r}; host supports {SENSOR_PLUGIN_API_VERSION!r}")
        if not self.outputs or len({port.name for port in self.outputs}) != len(self.outputs):
            raise ValueError("sensor plug-in outputs must be nonempty and uniquely named")
        if not self.supported_truth_modes or any(not mode.strip() for mode in self.supported_truth_modes):
            raise ValueError("sensor plug-in supported_truth_modes must contain nonempty values")
        if any(not capability.strip() for capability in self.required_truth):
            raise ValueError("sensor plug-in required_truth values must not be empty")
        if not self.sample_modes or not self.sample_modes <= {"instantaneous", "interval"}:
            raise ValueError("sensor plug-in sample_modes are invalid")
        if not math.isfinite(self.default_cadence_s) or self.default_cadence_s <= 0.0:
            raise ValueError("sensor plug-in default cadence must be positive and finite")
        ####

    ####


@dataclass(frozen=True, slots=True)
class SensorBuildContext:
    """Host-owned construction inputs that are independent of family config."""

    sensor_id: str
    seed: int
    body_from_sensor: Array3 = field(default_factory=lambda: np.eye(3))
    lever_arm_body_m: Array3 = field(default_factory=lambda: np.zeros(3))
    services: SensorServices = field(default_factory=SensorServices)
    resources: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.sensor_id.strip():
            raise ValueError("sensor build context sensor_id must not be empty")
        if isinstance(self.seed, bool) or not isinstance(self.seed, int):
            raise TypeError("sensor build context seed must be an integer")
        object.__setattr__(self, "body_from_sensor", _rotation(self.body_from_sensor, "body_from_sensor"))
        object.__setattr__(self, "lever_arm_body_m", _vector3(self.lever_arm_body_m, "lever_arm_body_m"))
        object.__setattr__(self, "resources", _immutable_mapping(self.resources))
        ####

    ####


class ContextSensorModel(Protocol):
    """Structural runtime interface for a context-aware sensor model."""

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[Any]: ...


SensorFactory = Callable[[BaseModel, SensorBuildContext], object]


@dataclass(frozen=True, slots=True)
class SensorPluginDescriptor:
    manifest: SensorPluginManifest
    config_model: type[BaseModel]
    factory: SensorFactory

    def validate_config(self, value: Mapping[str, object] | None = None) -> BaseModel:
        return self.config_model.model_validate(dict(value or {}))
        ####

    ####


class RegisteredSensorInstance:
    """Attach manifest/config identity to one family-owned implementation."""

    def __init__(self, descriptor: SensorPluginDescriptor, config: BaseModel, implementation: object) -> None:
        self.descriptor = descriptor
        self.manifest = descriptor.manifest
        self.config = config
        self.implementation = implementation
        implementation_provenance = getattr(implementation, "provenance", {})
        self.provenance = {
            **(dict(implementation_provenance) if isinstance(implementation_provenance, Mapping) else {}),
            "sensor_plugin": {
                "api_version": self.manifest.api_version,
                "kind": self.manifest.kind,
                "family": self.manifest.family,
                "language_kinds": sorted(self.manifest.language_kinds),
                "config": config.model_dump(mode="json"),
            },
        }

    def _annotate(self, packet: MeasurementPacket[Any]) -> MeasurementPacket[Any]:
        output = self.manifest.outputs[0]
        return replace(
            packet,
            port=packet.port if packet.port != "measurement" else output.name,
            schema_id=packet.schema_id or output.schema_id,
        )
        ####

    def sample(self, truth: TruthPoint) -> MeasurementPacket[Any]:
        sampler = getattr(self.implementation, "sample", None)
        if not callable(sampler):
            raise TypeError(f"sensor plug-in {self.manifest.kind!r} requires a SensorContext sample request")
        return self._annotate(sampler(truth))
        ####

    def sample_segment(self, segment: TruthSegment) -> MeasurementPacket[Any]:
        sampler = getattr(self.implementation, "sample_segment", None)
        packet = sampler(segment) if callable(sampler) else self.sample(segment.end)
        return self._annotate(packet)
        ####

    def sample_request(self, request: SensorSampleRequest) -> MeasurementPacket[Any]:
        context_sampler = getattr(self.implementation, "sample_request", None)
        if callable(context_sampler):
            return self._annotate(context_sampler(request))
        if request.point is not None:
            return self.sample(request.point.host)
        segment = cast(SensorContextSegment, request.segment)
        return self.sample_segment(TruthSegment(segment.start.host, segment.end.host))
        ####

    def reset(self) -> None:
        reset = getattr(self.implementation, "reset", None)
        if callable(reset):
            reset()
        ####

    def snapshot(self) -> Mapping[str, object]:
        snapshot = getattr(self.implementation, "snapshot", None)
        if not callable(snapshot):
            raise TypeError(f"sensor plug-in {self.manifest.kind!r} does not support checkpointing")
        value = snapshot()
        if isinstance(value, BaseModel):
            return value.model_dump(mode="json")
        if not isinstance(value, Mapping):
            raise TypeError("sensor plug-in snapshot must be a mapping or Pydantic model")
        return dict(value)
        ####

    def restore(self, checkpoint: Mapping[str, object]) -> None:
        restore = getattr(self.implementation, "restore", None)
        if not callable(restore):
            raise TypeError(f"sensor plug-in {self.manifest.kind!r} does not support checkpoint restoration")
        restore(checkpoint)
        ####

    ####


class SensorPluginRegistry:
    """Deterministic registry of validated sensor-family descriptors."""

    def __init__(self) -> None:
        self._descriptors: dict[str, SensorPluginDescriptor] = {}

    def register(self, descriptor: SensorPluginDescriptor) -> None:
        kind = descriptor.manifest.kind
        if kind in self._descriptors:
            raise ValueError(f"sensor plug-in kind {kind!r} is already registered")
        self._descriptors[kind] = descriptor
        ####

    def descriptor(self, kind: str) -> SensorPluginDescriptor:
        try:
            return self._descriptors[kind]
        except KeyError as exc:
            available = ", ".join(sorted(self._descriptors)) or "none"
            raise ValueError(f"unsupported sensor provider kind {kind!r}; available: {available}") from exc
        ####

    def manifests(self) -> tuple[SensorPluginManifest, ...]:
        return tuple(self._descriptors[kind].manifest for kind in sorted(self._descriptors))
        ####

    def validate_config(self, kind: str, value: Mapping[str, object] | None = None) -> BaseModel:
        return self.descriptor(kind).validate_config(value)
        ####

    def create(
        self,
        kind: str,
        value: Mapping[str, object] | None,
        context: SensorBuildContext,
    ) -> RegisteredSensorInstance:
        descriptor = self.descriptor(kind)
        config = descriptor.validate_config(value)
        implementation = descriptor.factory(config, context)
        return RegisteredSensorInstance(descriptor, config, implementation)
        ####

    ####


PayloadEncoder = Callable[[object], Mapping[str, object]]
PayloadDecoder = Callable[[Mapping[str, object]], object]


@dataclass(frozen=True, slots=True)
class PayloadCodec:
    """Versioned JSON codec and semantic contract for one payload type."""

    schema_id: str
    payload_type: type[object]
    encode: PayloadEncoder
    decode: PayloadDecoder
    contract: Mapping[str, object] = field(default_factory=dict)
    legacy_kind: str | None = None

    def __post_init__(self) -> None:
        if not self.schema_id.strip():
            raise ValueError("payload codec schema_id must not be empty")
        object.__setattr__(self, "contract", _immutable_mapping(self.contract))
        ####

    ####


class PayloadCodecRegistry:
    """Resolve packet serialization by schema identity instead of type switches."""

    def __init__(self) -> None:
        self._by_schema: dict[str, PayloadCodec] = {}
        self._by_type: dict[type[object], PayloadCodec] = {}
        self._by_legacy_kind: dict[str, PayloadCodec] = {}

    def register(self, codec: PayloadCodec) -> None:
        if codec.schema_id in self._by_schema:
            raise ValueError(f"payload schema {codec.schema_id!r} is already registered")
        if codec.payload_type in self._by_type:
            raise ValueError(f"payload type {codec.payload_type.__name__!r} already has a codec")
        if codec.legacy_kind is not None and codec.legacy_kind in self._by_legacy_kind:
            raise ValueError(f"legacy payload kind {codec.legacy_kind!r} is already registered")
        self._by_schema[codec.schema_id] = codec
        self._by_type[codec.payload_type] = codec
        if codec.legacy_kind is not None:
            self._by_legacy_kind[codec.legacy_kind] = codec
        ####

    def for_payload(self, payload: object) -> PayloadCodec:
        codec = self._by_type.get(type(payload))
        if codec is None:
            raise TypeError(f"no measurement payload codec is registered for {type(payload).__name__!r}")
        return codec
        ####

    def for_schema(self, schema_id: str) -> PayloadCodec:
        try:
            return self._by_schema[schema_id]
        except KeyError as exc:
            raise ValueError(f"unknown measurement payload schema {schema_id!r}") from exc
        ####

    def for_legacy_kind(self, kind: str) -> PayloadCodec:
        try:
            return self._by_legacy_kind[kind]
        except KeyError as exc:
            raise ValueError(f"unknown legacy measurement payload kind {kind!r}") from exc
        ####

    ####


_SENSOR_PLUGIN_REGISTRY = SensorPluginRegistry()
_PAYLOAD_CODEC_REGISTRY = PayloadCodecRegistry()
_BUILTINS_LOADED = False


def _load_builtin_sensor_plugins() -> None:
    global _BUILTINS_LOADED
    if _BUILTINS_LOADED:
        return
    # Set first so a plug-in module can safely ask for a registry while its
    # descriptor set is being constructed without recursively loading itself.
    _BUILTINS_LOADED = True
    from taoryx.sensor_plugins import register_builtin_sensor_plugins

    register_builtin_sensor_plugins(_SENSOR_PLUGIN_REGISTRY, _PAYLOAD_CODEC_REGISTRY)
    ####


def sensor_plugin_registry() -> SensorPluginRegistry:
    _load_builtin_sensor_plugins()
    return _SENSOR_PLUGIN_REGISTRY
    ####


def payload_codec_registry() -> PayloadCodecRegistry:
    _load_builtin_sensor_plugins()
    return _PAYLOAD_CODEC_REGISTRY
    ####


def packet_to_record(
    packet: MeasurementPacket[Any],
    *,
    provenance: PacketProvenanceInput | None = None,
) -> MeasurementPacketRecord:
    """Serialize a packet using its registered payload schema."""

    codec: PayloadCodec | None = None
    payload_record: PacketPayloadRecord | None = None
    if packet.payload is not None:
        codec = payload_codec_registry().for_payload(packet.payload)
        if packet.schema_id is not None and packet.schema_id != codec.schema_id:
            raise ValueError(f"packet schema {packet.schema_id!r} does not match payload codec {codec.schema_id!r}")
        payload_record = _packet_record_mapping(codec.encode(packet.payload), field_name="payload")
    elif packet.schema_id is not None:
        codec = payload_codec_registry().for_schema(packet.schema_id)
    schema_id = packet.schema_id if packet.schema_id is not None else (None if codec is None else codec.schema_id)
    record: MeasurementPacketRecord = {
        "sampled_at_s": packet.sampled_at_s,
        "available_at_s": packet.available_at_s,
        "interval_start_s": packet.interval_start_s,
        "valid": packet.valid,
        "payload": payload_record,
        "payload_contract": {} if codec is None else _packet_record_mapping(codec.contract, field_name="payload_contract"),
        "provenance": _packet_record_mapping(provenance or {}, field_name="provenance"),
    }
    if packet.sensor_id is not None:
        record["sensor_id"] = packet.sensor_id
    if packet.port != "measurement":
        record["port"] = packet.port
    if packet.sequence is not None:
        record["sequence"] = packet.sequence
    if schema_id is not None:
        record["schema_id"] = schema_id
    if packet.invalid_reason is not None:
        record["invalid_reason"] = packet.invalid_reason
    return record
    ####


def packet_from_record(record: MeasurementPacketRecord) -> MeasurementPacket[Any]:
    """Restore a packet encoded by :func:`packet_to_record` or schema-v1 checkpoints."""

    raw_payload = record.get("payload")
    schema_id_value = record.get("schema_id")
    schema_id = None if schema_id_value is None else str(schema_id_value)
    payload: object | None = None
    if raw_payload is not None:
        if not isinstance(raw_payload, Mapping):
            raise ValueError("measurement packet payload must be a mapping")
        if schema_id is not None:
            codec = payload_codec_registry().for_schema(schema_id)
        else:
            legacy_kind = raw_payload.get("kind")
            if legacy_kind is None:
                raise ValueError("measurement payload is missing schema_id and legacy kind")
            codec = payload_codec_registry().for_legacy_kind(str(legacy_kind))
            schema_id = codec.schema_id
        payload = codec.decode(raw_payload)
    elif schema_id is not None:
        payload_codec_registry().for_schema(schema_id)
    return MeasurementPacket(
        sampled_at_s=float(cast(float | int | str, record["sampled_at_s"])),
        available_at_s=float(cast(float | int | str, record["available_at_s"])),
        interval_start_s=(None if record.get("interval_start_s") is None else float(cast(float | int | str, record["interval_start_s"]))),
        payload=payload,
        valid=bool(record.get("valid", True)),
        sensor_id=None if record.get("sensor_id") is None else str(record["sensor_id"]),
        port=str(record.get("port", "measurement")),
        sequence=None if record.get("sequence") is None else int(cast(int | str, record["sequence"])),
        schema_id=schema_id,
        invalid_reason=None if record.get("invalid_reason") is None else str(record["invalid_reason"]),
    )
    ####


def _packet_record_mapping(value: Mapping[str, object], *, field_name: str) -> dict[str, object]:
    """Copy one codec-owned JSON object while preserving the packet boundary."""

    if any(not isinstance(key, str) for key in value):
        raise ValueError(f"measurement packet {field_name} requires string keys")
    return dict(value)
    ####


__all__ = [
    "ArrayFrameReference",
    "CommittedSceneQuery",
    "ContextSensorModel",
    "EntityTruth",
    "EnvironmentSnapshot",
    "MeasurementPacket",
    "MeasurementPacketRecord",
    "PayloadCodec",
    "PayloadCodecRegistry",
    "RegisteredSensorInstance",
    "SENSOR_PLUGIN_API_VERSION",
    "SensorArtifactStore",
    "SensorBuildContext",
    "SensorContext",
    "SensorContextSegment",
    "SensorOutputPort",
    "SensorPluginDescriptor",
    "SensorPluginManifest",
    "SensorPluginRegistry",
    "SensorSampleRequest",
    "SensorServices",
    "TruthPoint",
    "TruthSegment",
    "packet_from_record",
    "parse_measurement_packet_record",
    "packet_to_record",
    "payload_codec_registry",
    "sensor_plugin_registry",
]
####
